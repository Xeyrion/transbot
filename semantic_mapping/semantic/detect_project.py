#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对抽出来的每一帧跑检测，用深度和相机位姿投到 map/odom 系，攒成 observations.json。

    python3 detect_project.py <frames_dir> [--backend yolo26|cv2] [--conf 0.25]

两个后端：
  yolo26  Ultralytics YOLO26（默认 models/yolo26s.pt）。需要 python>=3.8 和 torch。
          控制机上用的是
          ~/miniconda3/envs/yolo/bin/python，不动系统 python3.6。
  cv2     原来的 OpenCV dnn + YOLOv3-tiny，只要 cv2，任何 python3 都能跑，
          当兜底用。

权重路径可用 SEMANTIC_WEIGHTS 覆盖；cv2 后端的模型在 SEMANTIC_MODELS 下。
"""
import argparse
import json
import os
import sys

import numpy as np
from PIL import Image
from depth_projection import project_depth
import hashlib
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from viewer_core import frame_metadata
from project_paths import models_dir

MODELS = os.environ.get('SEMANTIC_MODELS', models_dir())
NMS = 0.4


# ---------------------------------------------------------------- 检测后端
class Darknet(object):
    """OpenCV dnn + YOLOv3-tiny。不依赖 torch，检出率一般，胜在到处能跑。"""

    def __init__(self, conf):
        import cv2
        self.cv2 = cv2
        self.conf = conf
        self.net = cv2.dnn.readNetFromDarknet(os.path.join(MODELS, 'yolov3-tiny.cfg'),
                                              os.path.join(MODELS, 'yolov3-tiny.weights'))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        layers = self.net.getLayerNames()
        self.outs = [layers[int(i) - 1] for i in np.asarray(self.net.getUnconnectedOutLayers()).reshape(-1)]
        with open(os.path.join(MODELS, 'coco.names')) as f:
            self.names = [l.strip() for l in f if l.strip()]

    def __call__(self, bgr):
        cv2 = self.cv2
        h, w = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(bgr, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        boxes, scores, ids = [], [], []
        for layer in self.net.forward(self.outs):
            for det in layer:
                vals = det[5:]
                cls = int(np.argmax(vals))
                score = float(vals[cls] * det[4])
                if score < self.conf:
                    continue
                cx, cy, bw, bh = det[0] * w, det[1] * h, det[2] * w, det[3] * h
                boxes.append([int(cx - bw / 2), int(cy - bh / 2), int(bw), int(bh)])
                scores.append(score)
                ids.append(cls)
        keep = cv2.dnn.NMSBoxes(boxes, scores, self.conf, NMS)
        out = []
        for i in (keep.flatten() if len(keep) else []):
            x, y, bw, bh = boxes[int(i)]
            out.append((self.names[ids[int(i)]], scores[int(i)], x, y, bw, bh))
        return out


class Ultralytics(object):
    """YOLO26（或任意 ultralytics 模型）。端到端，不用自己跑 NMS。"""

    def __init__(self, conf):
        from ultralytics import YOLO
        weights = os.path.abspath(os.environ.get('SEMANTIC_WEIGHTS', os.path.join(MODELS, 'yolo26s.pt')))
        if not os.path.isfile(weights):
            raise ValueError('Local model weights are missing: ' + weights)
        self.model = YOLO(weights)
        self.conf = conf

    def __call__(self, bgr):
        result = self.model(bgr, conf=self.conf, retina_masks=True, verbose=False)[0]
        names = result.names
        out = []
        for index, box in enumerate(result.boxes):
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            record = dict(label=names[int(box.cls)], score=float(box.conf),
                          bbox_xywh=[int(x1), int(y1), int(x2-x1), int(y2-y1)])
            if result.masks is not None:
                mask = result.masks.data[index].cpu().numpy() > .5
                if mask.shape != bgr.shape[:2]:
                    raise ValueError('Model mask is not at native RGB resolution')
                record['_mask'] = mask
            out.append(record)
        return out


# ---------------------------------------------------------------- 投影
def project(depth, k, m, x, y, bw, bh, max_range=8.0, distortion=None, details=False):
    result = project_depth(depth,k,m,[x,y,bw,bh],distortion=distortion,max_range=max_range)
    if result['point'] is None:
        return None
    return (result['point'],result['quality']) if details else result['point']


def sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frames_dir')
    ap.add_argument('--backend', choices=('yolo26', 'cv2'), default='yolo26')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--out', default=None)
    ap.add_argument('--projection', choices=('center','surface','mask-surface'), default='center')
    ap.add_argument('--predictions', help='Reuse a validated predictions.json without running a model')
    args = ap.parse_args()

    sampling_path = os.path.join(args.frames_dir, 'sampling.json')
    sampling = json.load(open(sampling_path)) if os.path.isfile(sampling_path) else None
    if sampling and sampling['state'] != 'complete':
        raise ValueError('Refusing incomplete/failed frame extraction')
    cached=json.load(open(args.predictions)) if args.predictions else None
    if cached and args.conf < cached.get('confidence_floor',0.):
        raise ValueError('Cached detections do not cover the requested lower confidence threshold')
    detect=None if cached else (Ultralytics(args.conf) if args.backend == 'yolo26' else Darknet(args.conf))
    predictions=dict(schema_version=1,confidence_floor=args.conf,frames={})
    obs = []
    detections=[]
    frames = [os.path.dirname(path) for path in frame_metadata(args.frames_dir)]
    if not frames:
        raise ValueError('No frames to detect')
    world, geometry = None, None
    for i, frame_dir in enumerate(frames):
        rgb = np.asarray(Image.open(os.path.join(frame_dir, 'rgb.png')).convert('RGB'))
        bgr = np.ascontiguousarray(rgb[:, :, ::-1])
        depth = np.load(os.path.join(frame_dir, 'depth.npy'))
        meta = json.load(open(os.path.join(frame_dir, 'meta.json')))
        k, m = meta['k'], meta['t_map_cam']
        current_world = meta.get('world_frame', meta.get('root'))
        if current_world not in ('map', 'odom'):
            raise ValueError('Frame has no explicit world coordinate system')
        if world is not None and current_world != world:
            raise ValueError('Mixed world coordinate systems')
        world, geometry = current_world, meta.get('geometry')
        frame_id=os.path.basename(frame_dir)
        image_sha=sha256(os.path.join(frame_dir,'rgb.png'))
        if cached:
            saved=cached['frames'][frame_id]
            if saved['rgb_sha256']!=image_sha or abs(saved['rgb_record']-meta['rgb_record'])>1e-6:
                raise ValueError('Cached predictions belong to a different RGB frame: '+frame_id)
            found=saved['detections']
        else:
            found=detect(bgr)
        saved_detections=[]
        for index, raw in enumerate(found):
            if not isinstance(raw,dict):
                label,score,x,y,bw,bh=raw
                raw=dict(label=label,score=float(score),bbox_xywh=[x,y,bw,bh])
            raw=dict(raw);mask=raw.pop('_mask',None)
            if mask is not None:
                mask_dir=os.path.join(args.frames_dir,'masks');os.makedirs(mask_dir,exist_ok=True)
                raw['mask']=os.path.join('masks','%s_%03d.png'%(frame_id,index))
                mask_path=os.path.join(args.frames_dir,raw['mask'])
                Image.fromarray(mask.astype(np.uint8)*255).save(mask_path)
                raw['mask_sha256']=sha256(mask_path)
            elif raw.get('mask'):
                mask_path=os.path.abspath(os.path.join(os.path.dirname(args.predictions),raw['mask']))
                if sha256(mask_path)!=raw['mask_sha256']:raise ValueError('Cached instance mask changed')
                mask=np.asarray(Image.open(mask_path).convert('L'))>0
                raw['mask']=os.path.relpath(mask_path,args.frames_dir)
            saved_detections.append(raw)
            label,score=raw['label'],raw['score']
            if score < args.conf:
                continue
            record=dict(raw,frame=i,source_frame=frame_id)
            detections.append(record)
            result=project_depth(depth,k,m,raw['bbox_xywh'],mode=args.projection,
                                 instance_mask=mask,distortion=meta.get('distortion_rgb'))
            record['depth_diagnostics']=result['quality']
            if result['point'] is None:
                record.update(projection='rejected',reason=result['reason'])
                continue
            p,quality=result['point'],result['quality']
            obs.append(dict(label=label, score=round(score,3), t=round(meta['stamp'],3),
                            x=round(p[0],3), y=round(p[1],3), z=round(p[2],3),
                            dist=round(p[3],2), frame=i,source_frame=frame_id,
                            depth_stamp=meta.get('depth_stamp'),pair_delta_ms=meta.get('pair_delta_ms'),
                            camera_xyz=[m[j][3] for j in range(3)],
                            bbox_xywh=raw['bbox_xywh'],
                            view_direction=[m[j][2] for j in range(3)]))
            obs[-1]['depth_quality']=quality
            obs[-1]['observation_id']='%s:%d'%(frame_id,len(obs))
            record.update(projection='accepted',observation_id=obs[-1]['observation_id'])
        predictions['frames'][frame_id]=dict(rgb_sha256=image_sha,rgb_record=meta['rgb_record'],detections=saved_detections)
        if (i + 1) % 20 == 0:
            print('  已处理 %d/%d 帧，累计 %d 个观测' % (i + 1, len(frames), len(obs)))

    if not cached:
        with open(os.path.join(args.frames_dir,'predictions.json'),'w') as f:json.dump(predictions,f,indent=1)
    out = args.out or os.path.join(args.frames_dir, 'observations.json')
    policy=dict(method=args.projection,min_valid_fraction=.25)
    if args.projection=='center':
        policy.update(min_surface_support=.7,max_mad_m='max(0.1,0.1*z)')
    else:
        policy.update(min_connected_support_fraction=.2,depth_bin_m=.08,
                      band_half_width_m='max(0.08,0.04*z)',max_competing_support_ratio=.8)
    with open(out, 'w') as f:
        json.dump(dict(schema_version=2, world_frame=world, geometry=geometry,
                       frames=len(frames), backend=args.backend, observations=obs,detections=detections,
                       depth_policy=policy), f, indent=1)
    kinds = {}
    for o in obs:
        kinds[o['label']] = kinds.get(o['label'], 0) + 1
    print('%d 帧 → %d 个观测 %s' % (len(frames), len(obs),
                                   dict(sorted(kinds.items(), key=lambda kv: -kv[1]))))
    print('→ ' + out)


if __name__ == '__main__':
    main()

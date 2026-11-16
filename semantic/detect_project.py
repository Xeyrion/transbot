#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对抽出来的每一帧跑检测，把框投到 map 系，攒成 observations.json。

    python3 detect_project.py <frames_dir> [--backend yolo26|cv2] [--conf 0.25]

两个后端：
  yolo26  Ultralytics YOLO26（默认 yolo26n.pt，端到端无 NMS，CPU 上比 yolo11n 快
          最多 43%）。需要 python>=3.8 和 torch —— 控制机上用的是
          ~/miniconda3/envs/yolo/bin/python，不动系统 python3.6。
  cv2     原来的 OpenCV dnn + YOLOv3-tiny，只要 cv2，任何 python3 都能跑，
          当兜底用。

权重路径可用 SEMANTIC_WEIGHTS 覆盖；cv2 后端的模型在 SEMANTIC_MODELS 下。
"""
import argparse
import glob
import json
import os

import numpy as np
from PIL import Image
from camera_geometry import undistort

MODELS = os.environ.get('SEMANTIC_MODELS',
                        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'models'))
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
        weights = os.path.abspath(os.environ.get('SEMANTIC_WEIGHTS',
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'yolo26n.pt')))
        if not os.path.isfile(weights):
            raise ValueError('Local model weights are missing: ' + weights)
        self.model = YOLO(weights)
        self.conf = conf

    def __call__(self, bgr):
        result = self.model(bgr, conf=self.conf, verbose=False)[0]
        names = result.names
        out = []
        for box in result.boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            out.append((names[int(box.cls)], float(box.conf),
                        int(x1), int(y1), int(x2 - x1), int(y2 - y1)))
        return out


# ---------------------------------------------------------------- 投影
def project(depth, k, m, x, y, bw, bh, max_range=8.0, distortion=None, details=False):
    """框中心附近取深度中位数，反投影成 map 坐标。"""
    h, w = depth.shape[:2]
    u0, u1 = max(0, x + bw // 4), min(w, x + 3 * bw // 4)
    v0, v1 = max(0, y + bh // 4), min(h, y + 3 * bh // 4)
    if u1 <= u0 or v1 <= v0 or not (0 <= x+bw/2.0 < w and 0 <= y+bh/2.0 < h):
        return None
    patch = depth[v0:v1, u0:u1]
    valid = patch[(patch > 0.2) & (patch < max_range)]
    if valid.size < 10 or float(valid.size)/max(1,patch.size)<.25:
        return None
    z = float(np.median(valid))
    mad=float(np.median(np.abs(valid-z)))
    if mad>max(.1,.1*z): return None
    mask=np.isfinite(patch)&(patch>.2)&(patch<max_range)&(np.abs(patch-z)<=max(.03,3*1.4826*mad))
    rows,cols=np.nonzero(mask)
    if len(rows)<10 or float(len(rows))/valid.size<.7: return None
    cu,cv_=float(np.median(cols))+u0,float(np.median(rows))+v0
    rx,ry=undistort((cu-k[2])/k[0],(cv_-k[5])/k[4],distortion)
    if not np.isfinite(rx) or not np.isfinite(ry): return None
    p = [float(rx)*z,float(ry)*z,z]
    result=[sum(m[i][j] * p[j] for j in range(3)) + m[i][3] for i in range(3)] + [z]
    quality=dict(depth_mad_m=mad,valid_fraction=float(valid.size)/patch.size,
                 support_pixels=len(rows),sigma_m=max(.02,1.4826*mad),
                 uncertainty_scope='depth_dispersion_only; excludes calibration and pose error',
                 position_semantics='visible_surface_sample; not object centre')
    return (result,quality) if details else result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frames_dir')
    ap.add_argument('--backend', choices=('yolo26', 'cv2'), default='yolo26')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    sampling_path = os.path.join(args.frames_dir, 'sampling.json')
    sampling = json.load(open(sampling_path)) if os.path.isfile(sampling_path) else None
    if sampling and sampling['state'] != 'complete':
        raise ValueError('Refusing incomplete/failed frame extraction')
    detect = Ultralytics(args.conf) if args.backend == 'yolo26' else Darknet(args.conf)
    obs = []
    detections=[]
    frames = sorted(glob.glob(os.path.join(args.frames_dir, '[0-9][0-9][0-9][0-9]')))
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
        for label, score, x, y, bw, bh in detect(bgr):
            record=dict(label=label,score=float(score),bbox_xywh=[x,y,bw,bh],frame=i,source_frame=os.path.basename(frame_dir))
            detections.append(record)
            projected = project(depth,k,m,x,y,bw,bh,distortion=meta.get('distortion_rgb'),details=True)
            if projected is None:
                record.update(projection='rejected',reason='depth_support_spread_or_invalid_ray')
                continue
            p,quality=projected
            obs.append(dict(label=label, score=round(score, 3), t=round(meta['stamp'], 3),
                            x=round(p[0], 3), y=round(p[1], 3), z=round(p[2], 3),
                            dist=round(p[3], 2), frame=i,
                            source_frame=os.path.basename(frame_dir),
                            depth_stamp=meta.get('depth_stamp'), pair_delta_ms=meta.get('pair_delta_ms')))
            obs[-1]['depth_quality']=quality
            obs[-1]['observation_id']='%s:%d'%(os.path.basename(frame_dir),len(obs))
            record.update(projection='accepted',observation_id=obs[-1]['observation_id'])
        if (i + 1) % 20 == 0:
            print('  已处理 %d/%d 帧，累计 %d 个观测' % (i + 1, len(frames), len(obs)))

    out = args.out or os.path.join(args.frames_dir, 'observations.json')
    with open(out, 'w') as f:
        json.dump(dict(schema_version=2, world_frame=world, geometry=geometry,
                       frames=len(frames), backend=args.backend, observations=obs,detections=detections,
                       depth_policy=dict(min_valid_fraction=.25,min_surface_support=.7,
                                         max_mad_m='max(0.1,0.1*z)')), f, indent=1)
    kinds = {}
    for o in obs:
        kinds[o['label']] = kinds.get(o['label'], 0) + 1
    print('%d 帧 → %d 个观测 %s' % (len(frames), len(obs),
                                   dict(sorted(kinds.items(), key=lambda kv: -kv[1]))))
    print('→ ' + out)


if __name__ == '__main__':
    main()

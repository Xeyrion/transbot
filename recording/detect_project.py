# -*- coding: utf-8 -*-
"""在抽出来的一帧上跑 YOLO，把检测框投到 map 系（py3，cv2.dnn，不需要 torch）。

    python3 detect_project.py <frame_dir> <out.png>

frame_dir 是 extract_frame.py 的产物（rgb.png / depth.npy / meta.json）。
输出左图是带框的彩色帧，右图是俯视图上的检测位置。
"""
import json
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw

MODELS = os.path.expanduser('~/robot_research/bagtools/models')
CONF = 0.25
NMS = 0.4


def load_net():
    net = cv2.dnn.readNetFromDarknet(os.path.join(MODELS, 'yolov3-tiny.cfg'),
                                     os.path.join(MODELS, 'yolov3-tiny.weights'))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    layers = net.getLayerNames()
    out = [layers[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    with open(os.path.join(MODELS, 'coco.names')) as f:
        names = [l.strip() for l in f if l.strip()]
    return net, out, names


def detect(net, outs, names, rgb):
    h, w = rgb.shape[:2]
    blob = cv2.dnn.blobFromImage(rgb, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    outputs = net.forward(outs)
    boxes, scores, ids = [], [], []
    for layer in outputs:
        for det in layer:
            vals = det[5:]
            cls = int(np.argmax(vals))
            score = float(vals[cls] * det[4])
            if score < CONF:
                continue
            cx, cy, bw, bh = det[0] * w, det[1] * h, det[2] * w, det[3] * h
            boxes.append([int(cx - bw / 2), int(cy - bh / 2), int(bw), int(bh)])
            scores.append(score)
            ids.append(cls)
    keep = cv2.dnn.NMSBoxes(boxes, scores, CONF, NMS)
    result = []
    for i in (keep.flatten() if len(keep) else []):
        x, y, bw, bh = boxes[int(i)]
        result.append((names[ids[int(i)]], scores[int(i)], x, y, bw, bh))
    return result


def main():
    frame_dir, out = sys.argv[1], sys.argv[2]
    rgb = np.asarray(Image.open(os.path.join(frame_dir, 'rgb.png')).convert('RGB'))
    depth = np.load(os.path.join(frame_dir, 'depth.npy'))
    meta = json.load(open(os.path.join(frame_dir, 'meta.json')))
    k = meta['k']
    t_map_cam = np.array(meta['t_map_cam'])

    net, outs, names = load_net()
    dets = detect(net, outs, names, rgb)
    print('检出 %d 个：%s' % (len(dets), ', '.join('%s %.2f' % (d[0], d[1]) for d in dets)))

    left = Image.fromarray(rgb)
    dl = ImageDraw.Draw(left)
    h, w = depth.shape[:2]
    projected = []
    for label, score, x, y, bw, bh in dets:
        dl.rectangle([x, y, x + bw, y + bh], outline=(255, 60, 60), width=2)
        dl.text((x + 3, max(0, y - 12)), '%s %.2f' % (label, score), fill=(255, 255, 0))
        # 框中心附近取深度中位数，避开个别空洞
        u0, u1 = max(0, x + bw // 4), min(w, x + 3 * bw // 4)
        v0, v1 = max(0, y + bh // 4), min(h, y + 3 * bh // 4)
        patch = depth[v0:v1, u0:u1]
        valid = patch[(patch > 0.2) & (patch < 8.0)]
        if valid.size < 10:
            continue
        z = float(np.median(valid))
        cu, cv = x + bw / 2.0, y + bh / 2.0
        p_cam = np.array([(cu - k[2]) * z / k[0], (cv - k[5]) * z / k[4], z, 1.0])
        p_map = t_map_cam.dot(p_cam)[:3]
        projected.append((label, score, p_map, z))

    # 右图：俯视图
    side, ppm = 640, 40.0
    right = Image.new('RGB', (side, side), (13, 17, 23))
    dr = ImageDraw.Draw(right)
    cx, cy = t_map_cam[0, 3], t_map_cam[1, 3]
    for label, score, p, z in projected:
        px = side / 2.0 + (p[0] - cx) * ppm
        py = side / 2.0 - (p[1] - cy) * ppm
        dr.ellipse([px - 5, py - 5, px + 5, py + 5], fill=(255, 60, 60))
        dr.text((px + 8, py - 6), '%s %.1fm' % (label, z), fill=(255, 255, 0))
        dr.line([side / 2.0, side / 2.0, px, py], fill=(90, 90, 90))
        print('  %-12s 距离 %.2f m → map (%.2f, %.2f, %.2f)' % (label, z, p[0], p[1], p[2]))
    dr.ellipse([side / 2.0 - 5, side / 2.0 - 5, side / 2.0 + 5, side / 2.0 + 5], fill=(245, 158, 11))
    dr.text((6, 6), 'robot=orange  detections=red  (top-down, 16m)', fill=(210, 210, 210))

    canvas = Image.new('RGB', (left.width + side, max(left.height, side)), (0, 0, 0))
    canvas.paste(left, (0, 0))
    canvas.paste(right, (left.width, 0))
    canvas.save(out)
    print('→', out)


if __name__ == '__main__':
    main()

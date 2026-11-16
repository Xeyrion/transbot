# -*- coding: utf-8 -*-
"""从一个 bag 里抽一帧出来，供后面的检测/投影用（rosbag 只有 py2）。

    python2 extract_frame.py <bag> <outdir> [--t 秒]

产出：
    rgb.png     彩色帧
    depth.npy   深度（米，float32，和彩色同尺寸；未对齐的包会重投影）
    meta.json   内参、map←camera 的 4x4、时间戳
"""
import argparse
import json
import os

import numpy as np
import rosbag
from PIL import Image

import project_map as pm

RGB = '/camera/rgb/image_raw'
DEPTH = '/camera/depth/image_raw'
DEPTH_INFO = '/camera/depth/camera_info'
RGB_INFO = '/camera/rgb/camera_info'


def depth_array(msg):
    if msg.encoding in ('16UC1', 'mono16'):
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
        return d[:, :msg.width].astype(np.float32) / 1000.0
    d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
    return d[:, :msg.width].copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('outdir')
    ap.add_argument('--t', type=float, default=None, help='相对包起点的秒数')
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    start = bag.get_start_time()
    t = start + (args.t if args.t is not None else (bag.get_end_time() - start) / 2.0)

    tf = pm.Tf()
    tf.load(bag)
    for key, (xyz, yaw) in pm.FILL_INS.items():
        if key not in tf.edges:
            tf.edges[key] = [(0.0, pm.transform(xyz, pm.yaw_quat(yaw)))]
            tf.parent[key[1]] = key[0]

    def first_after(topic, when):
        for _t, m, _ in bag.read_messages(topics=[topic],
                                          start_time=__import__('rospy').Time.from_sec(when)):
            return m
        return None

    rgb = first_after(RGB, t)
    if rgb is None:
        raise SystemExit('没有彩色帧')
    s = rgb.header.stamp.to_sec()
    depth = first_after(DEPTH, s - 0.1)
    depth_info = first_after(DEPTH_INFO, s - 0.1)
    rgb_info = first_after(RGB_INFO, s - 0.1)
    bag.close()
    if depth is None or depth_info is None or rgb_info is None:
        raise SystemExit('缺深度或内参')

    d = depth_array(depth)
    aligned = depth.header.frame_id.strip('/') == 'camera_color_optical_frame'
    k = list(rgb_info.K) if aligned else list(depth_info.K)
    if not aligned and d.shape[:2] != (rgb.height, rgb.width):
        d = np.asarray(Image.fromarray(d).resize((rgb.width, rgb.height), Image.NEAREST))

    t_map_cam = tf.chain_to(depth.header.frame_id.strip('/'), depth.header.stamp.to_sec())
    if t_map_cam is None:
        raise SystemExit('map←camera 链断了')

    os.path.isdir(args.outdir) or os.makedirs(args.outdir)
    img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.step)[:, :rgb.width * 3]
    Image.fromarray(img.reshape(rgb.height, rgb.width, 3)).save(os.path.join(args.outdir, 'rgb.png'))
    np.save(os.path.join(args.outdir, 'depth.npy'), d)
    with open(os.path.join(args.outdir, 'meta.json'), 'w') as f:
        json.dump(dict(k=k, aligned=bool(aligned), frame=depth.header.frame_id.strip('/'),
                       t_map_cam=t_map_cam.tolist(), stamp=s, bag=os.path.basename(args.bag)), f)
    print('抽帧完成：%s  aligned=%s  内参 fx=%.2f' % (args.outdir, aligned, k[0]))


if __name__ == '__main__':
    main()

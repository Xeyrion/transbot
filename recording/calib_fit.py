# -*- coding: utf-8 -*-
"""定量评估相机外参：深度点云到激光点云的最近邻距离，并做小范围搜索。

    python2 calib_fit.py <bag>

先用单位变换跑一遍给出中位误差，再在 dx/dy/yaw 上网格搜索找最优值。
"""
import argparse
import math

import numpy as np
import rosbag

from calib_check import (DEPTH, INFO, SCAN, camera_chain, chain, depth_points,
                         matrix, nearest_scan, read_transforms)


def lidar_xy(scan, t_laser, limit=8.0):
    ang = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    r = np.asarray(scan.ranges, dtype=np.float32)
    ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max, limit))
    p = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]),
                  np.zeros(int(ok.sum()))], axis=1)
    q = p.dot(t_laser[:3, :3].T) + t_laser[:3, 3]
    return q[:, :2]


def camera_xy(depth, k, t_cam):
    pts = depth_points(depth, k, stride=4)
    base = pts.dot(t_cam[:3, :3].T) + t_cam[:3, 3]
    keep = base[(base[:, 2] > -0.04) & (base[:, 2] < 0.9)]
    return keep[:, :2]


def distances(cam, lidar):
    """每个深度点到最近激光点的距离。"""
    d = np.sqrt(((cam[:, None, :] - lidar[None, :, :]) ** 2).sum(axis=2))
    return d.min(axis=1)


def score(cam, lidar):
    d = distances(cam, lidar)
    return float(np.median(d)), float(np.percentile(d, 90))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--k', choices=('depth', 'rgb'), default='depth',
                    help='用哪套内参：对齐到彩色的深度图应该用 rgb 的')
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    table = read_transforms(bag)
    k = depth = scan = None
    for _t, m, _ in bag.read_messages(topics=[INFO]):
        depth_k = list(m.K)
        break
    rgb_k = None
    for _t, m, _ in bag.read_messages(topics=['/camera/rgb/camera_info']):
        rgb_k = list(m.K)
        break
    k = rgb_k if args.k == 'rgb' else depth_k
    for _t, m, _ in bag.read_messages(topics=[DEPTH]):
        depth = m
        break
    scan, dt = nearest_scan(bag, depth.header.stamp.to_sec())
    print('深度/激光时间差 %.0f ms' % (dt * 1000))
    bag.close()

    print('深度图 %dx%d frame=%s 用 %s 内参 fx=%.2f' % (depth.width, depth.height,
          depth.header.frame_id, args.k, k[0]))
    print('depth K fx=%.2f cx=%.2f / rgb K fx=%.2f cx=%.2f'
          % (depth_k[0], depth_k[2], rgb_k[0] if rgb_k else 0, rgb_k[2] if rgb_k else 0))
    lidar = lidar_xy(scan, table[('base_link', 'laser')])
    print('激光点 %d' % len(lidar))

    def evaluate(dx, dy, yaw):
        bias = matrix((dx, dy, 0.0), (0, 0, math.sin(math.radians(yaw) / 2),
                                      math.cos(math.radians(yaw) / 2)))
        t = chain(table, camera_chain(table), extra=bias)
        cam = camera_xy(depth, k, t)
        if len(cam) > 2500:                      # 搜索时降采样，够统计就行
            cam = cam[::max(1, len(cam) // 2500)]
        med, p90 = score(cam, lidar)
        return med, p90, len(cam)

    med, p90, n = evaluate(0, 0, 0)
    print('\n单位变换：深度点 %d，最近邻距离 中位 %.3f m，90分位 %.3f m' % (n, med, p90))

    best = (med, 0, 0, 0)
    for dx in (-0.10, -0.05, 0.0, 0.05, 0.10):
        for dy in (-0.10, -0.05, 0.0, 0.05, 0.10):
            for yaw in (-6, -3, 0, 3, 6):
                m, _p, _n = evaluate(dx, dy, yaw)
                if m < best[0]:
                    best = (m, dx, dy, yaw)
    print('粗搜索最优：dx=%+.2f dy=%+.2f yaw=%+.0f° → 中位 %.3f m'
          % (best[1], best[2], best[3], best[0]))
    for step in (0.02, 0.01):
        cx, cy, cyaw = best[1], best[2], best[3]
        for dx in (cx - step, cx, cx + step):
            for dy in (cy - step, cy, cy + step):
                for yaw in (cyaw - step * 100, cyaw, cyaw + step * 100):
                    m, _p, _n = evaluate(dx, dy, yaw)
                    if m < best[0]:
                        best = (m, dx, dy, yaw)
        print('细化 step=%.2f：dx=%+.3f dy=%+.3f yaw=%+.1f° → 中位 %.3f m，90分位 %.3f m'
              % (step, best[1], best[2], best[3], best[0],
                 evaluate(best[1], best[2], best[3])[1]))


if __name__ == '__main__':
    main()

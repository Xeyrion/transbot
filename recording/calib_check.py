# -*- coding: utf-8 -*-
"""验证相机外参：把深度点云和激光点云投到同一张俯视图上对比。

深度相机和激光看到的是同一个房间。外参对，两片点云就贴在一起；差多少一眼
能看出来 —— 不需要真值，用两路传感器互相印证。

    python2 calib_check.py <bag> <out.png> [--yaw 度] [--dx 米] [--dy 米] [--dz 米]

`--d?/--yaw` 是给 `astra_link → camera_link` 这一节加偏置，用来试候选值；
默认全 0，也就是假设两帧重合（URDF 里 astra_link 就是相机安装点）。
"""
import argparse
import math

import numpy as np
import rosbag
from PIL import Image, ImageDraw

DEPTH = '/camera/depth/image_raw'
INFO = '/camera/depth/camera_info'
SCAN = '/scan'

# base_link 往下的链。11-10 那批用 transbot_record.launch 录的包里有
# base_link → camera_link 这一条直连（launch 里的 static_transform_publisher），
# 11-12 那批是在导航栈跑着的时候录的，少了这条，只剩 base_link → astra_link。
TO_CAMERA_DIRECT = [('base_link', 'camera_link'),
                    ('camera_link', 'camera_depth_frame'),
                    ('camera_depth_frame', 'camera_depth_optical_frame')]
TO_CAMERA_FALLBACK = [('base_link', 'astra_link'), ('astra_link', 'camera_link'),
                      ('camera_link', 'camera_depth_frame'),
                      ('camera_depth_frame', 'camera_depth_optical_frame')]


def quat_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix(xyz, quat):
    t = np.eye(4)
    t[:3, :3] = quat_matrix(*quat)
    t[:3, 3] = xyz
    return t


def camera_chain(table):
    """包里有直连就用直连，否则走 astra_link 中转（那一段可能要靠偏置补）。"""
    return TO_CAMERA_DIRECT if ('base_link', 'camera_link') in table else TO_CAMERA_FALLBACK


def read_transforms(bag):
    """(parent, child) → 4x4。静态优先；动态只取第一条（这一帧内够用）。"""
    table = {}
    for topic in ('/tf_static', '/tf'):
        for _t, msg, _ in bag.read_messages(topics=[topic]):
            for x in msg.transforms:
                key = (x.header.frame_id.strip('/'), x.child_frame_id.strip('/'))
                if key not in table or topic == '/tf_static':
                    tr = x.transform
                    table[key] = matrix(
                        (tr.translation.x, tr.translation.y, tr.translation.z),
                        (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))
    return table


def chain(table, links, extra=None):
    """沿 (parent, child) 链右乘。extra 给 astra_link→camera_link 加偏置。"""
    t = np.eye(4)
    for key in links:
        if key == ('astra_link', 'camera_link'):
            if extra is not None:
                t = t.dot(extra)
                continue
            if key not in table:
                continue          # 缺这一节时按单位变换处理，正是要试的假设
        if key not in table:
            raise SystemExit('包里缺变换：%s → %s' % key)
        t = t.dot(table[key])
    return t


def nearest_scan(bag, stamp):
    """时间上离深度帧最近的激光帧。

    深度和激光各自带着自己的时间戳，机器人一动，两者所在的 base_link 就不是
    同一个位姿 —— 直接比会得到几十厘米的假误差。必须挑同一时刻的。
    """
    best, best_dt = None, 1e9
    for _t, msg, _ in bag.read_messages(topics=[SCAN]):
        dt = abs(msg.header.stamp.to_sec() - stamp)
        if dt < best_dt:
            best, best_dt = msg, dt
        elif dt > best_dt + 1.0:
            break                 # 时间戳单调，已经走远了
    return best, best_dt


def depth_points(msg, k, stride=3, max_range=6.0):
    """深度图 → 光心坐标系 3D 点（x 右、y 下、z 前）。"""
    if msg.encoding in ('16UC1', 'mono16'):
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
        d = d[:, :msg.width].astype(np.float32) / 1000.0
    else:
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
        d = d[:, :msg.width]
    v, u = np.mgrid[0:msg.height:stride, 0:msg.width:stride]
    z = d[0:msg.height:stride, 0:msg.width:stride]
    ok = np.isfinite(z) & (z > 0.2) & (z < max_range)
    z = z[ok]
    return np.stack([(u[ok] - k[2]) * z / k[0],
                     (v[ok] - k[5]) * z / k[4], z], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--yaw', type=float, default=0.0, help='astra→camera_link 偏航（度）')
    ap.add_argument('--dx', type=float, default=0.0)
    ap.add_argument('--dy', type=float, default=0.0)
    ap.add_argument('--dz', type=float, default=0.0)
    ap.add_argument('--ppm', type=float, default=60.0)
    ap.add_argument('--span', type=float, default=10.0)
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    table = read_transforms(bag)
    k = depth = scan = None
    for _t, m, _ in bag.read_messages(topics=[INFO]):
        k = list(m.K)
        break
    for _t, m, _ in bag.read_messages(topics=[DEPTH]):
        depth = m
        break
    if depth is not None:
        scan, dt = nearest_scan(bag, depth.header.stamp.to_sec())
        print('深度/激光时间差 %.0f ms' % (dt * 1000))
    bag.close()
    if k is None or depth is None:
        raise SystemExit('包里没有深度图或内参')

    bias = matrix((args.dx, args.dy, args.dz), (0, 0, math.sin(math.radians(args.yaw) / 2),
                                                math.cos(math.radians(args.yaw) / 2)))
    t_cam = chain(table, camera_chain(table), extra=bias)
    pts = depth_points(depth, k)
    base = pts.dot(t_cam[:3, :3].T) + t_cam[:3, 3]

    span, ppm = args.span, args.ppm
    side = int(span * ppm)
    img = Image.new('RGB', (side, side), (13, 17, 23))
    d = ImageDraw.Draw(img)

    def to_px(x, y):
        return side / 2.0 + x * ppm, side / 2.0 - y * ppm

    # 只留地面以上的点：地平面在 base_link 下 8 cm，去掉它才不会糊成一片
    keep = base[(base[:, 2] > -0.04) & (base[:, 2] < 0.9)]
    for x, y, _z in keep:
        px, py = to_px(x, y)
        if 0 <= px < side and 0 <= py < side:
            d.point((px, py), fill=(239, 68, 68))

    lidar = 0
    if scan is not None and ('base_link', 'laser') in table:
        t_laser = table[('base_link', 'laser')]
        ang = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        r = np.asarray(scan.ranges, dtype=np.float32)
        ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max, span / 2.0))
        p = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]),
                      np.zeros(int(ok.sum()))], axis=1)
        for x, y, _z in p.dot(t_laser[:3, :3].T) + t_laser[:3, 3]:
            px, py = to_px(x, y)
            if 0 <= px < side and 0 <= py < side:
                d.point((px, py), fill=(56, 189, 248))
                lidar += 1

    cx, cy = to_px(0.0, 0.0)
    d.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=(245, 158, 11))
    d.line([cx, cy, cx + 45, cy], fill=(150, 150, 150))
    d.text((6, 6), 'red=depth  blue=lidar  bias yaw=%+.1f dx=%+.2f dy=%+.2f dz=%+.2f'
           % (args.yaw, args.dx, args.dy, args.dz), fill=(210, 210, 210))
    img.save(args.out)
    print('depth=%d (kept %d) lidar=%d -> %s' % (len(base), len(keep), lidar, args.out))


if __name__ == '__main__':
    main()

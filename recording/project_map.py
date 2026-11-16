# -*- coding: utf-8 -*-
"""把深度点云投到 map 系，和激光叠在一起，检查整条坐标链通不通。

不涉及任何识别 —— 这一步只回答一个问题：一个像素，能不能被放到地图上正确
的位置。深度相机和激光看到的是同一面墙，两者叠起来对齐，说明链是通的。

    python2 project_map.py <bag> <out.png> [--frames 8]

包里有 base_link→camera_link 直连的（transbot_record.launch 录的那批）直接走；
没有的（导航栈跑着时录的）用 launch 里的数值补上。
"""
import argparse
import math

import numpy as np
import rosbag
from PIL import Image, ImageDraw

DEPTH = '/camera/depth/image_raw'
INFO = '/camera/depth/camera_info'
SCAN = '/scan'

# 静态环节的已知值。每批包缺的还不一样，所以不能假设"链是完整的"：
#   base_footprint→base_link  轮子高度，URDF 里的固定关节（11-12 的包有，11-10 的没有）
#   base_link→camera_link     相机安装，transbot_record.launch 里那条
# 缺哪个就按这里的值补，都是常量，补进去和真值等价。
FILL_INS = {
    ('base_footprint', 'base_link'): ((0.0, 0.0, 0.08), 0.0),
    ('base_link', 'camera_link'): ((0.08, 0.0, 0.10403), -0.05),
}


def quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform(txyz, quat):
    t = np.eye(4)
    t[:3, :3] = quat_matrix(quat)
    t[:3, 3] = txyz
    return t


def yaw_quat(rad):
    return (0.0, 0.0, math.sin(rad / 2.0), math.cos(rad / 2.0))


class Tf:
    """极简 3D TF：每个 (parent, child) 存一串 (stamp, 4x4)，查最近的一条。"""

    def __init__(self):
        self.edges = {}

    def load(self, bag):
        for topic in ('/tf_static', '/tf'):
            for _t, msg, _ in bag.read_messages(topics=[topic]):
                for x in msg.transforms:
                    key = (x.header.frame_id.strip('/'), x.child_frame_id.strip('/'))
                    tr = x.transform
                    stamp = x.header.stamp.to_sec()
                    self.edges.setdefault(key, []).append((
                        stamp,
                        transform((tr.translation.x, tr.translation.y, tr.translation.z),
                                  (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))))
        for key in self.edges:
            self.edges[key].sort(key=lambda e: e[0])
        self.parent = {child: parent for parent, child in self.edges}

    def at(self, key, stamp):
        entries = self.edges.get(key)
        if not entries:
            return None
        return min(entries, key=lambda e: abs(e[0] - stamp))[1]

    def chain_to(self, frame, stamp, root='map'):
        """root ← frame 的 4x4；中间缺链返回 None。"""
        t = np.eye(4)
        cur = frame
        while cur != root:
            parent = self.parent.get(cur)
            if parent is None:
                return None
            step = self.at((parent, cur), stamp)
            if step is None:
                return None
            t = step.dot(t)
            cur = parent
        return t


def depth_points(msg, k, stride=4, max_range=6.0):
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
    ap.add_argument('--frames', type=int, default=8)
    ap.add_argument('--stride', type=int, default=4)
    ap.add_argument('--ppm', type=float, default=40.0)
    ap.add_argument('--half', type=float, default=8.0, help='画布半宽（米）')
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    tf = Tf()
    tf.load(bag)
    for key, (xyz, yaw) in FILL_INS.items():
        if key not in tf.edges:
            tf.edges[key] = [(0.0, transform(xyz, yaw_quat(yaw)))]
            tf.parent[key[1]] = key[0]
            print('包里缺 %s→%s，按已知值补上' % key)

    k = None
    for _t, m, _ in bag.read_messages(topics=[INFO]):
        k = list(m.K)
        break
    rgb_k = None
    for _t, m, _ in bag.read_messages(topics=['/camera/rgb/camera_info']):
        rgb_k = list(m.K)
        break

    # 深度图若已对齐到彩色帧，几何就按彩色的内参算
    depth_frame = None
    for _t, m, _ in bag.read_messages(topics=[DEPTH]):
        depth_frame = m.header.frame_id.strip('/')
        break
    if depth_frame == 'camera_color_optical_frame' and rgb_k:
        k = rgb_k
        print('深度已对齐到彩色帧，用彩色内参 fx=%.2f' % k[0])

    start = bag.get_start_time()
    span = bag.get_end_time() - start
    times = [start + span * (i + 1) / (args.frames + 1.0) for i in range(args.frames)]

    ppm, half = args.ppm, args.half
    side = int(half * 2 * ppm)
    img = Image.new('RGB', (side, side), (13, 17, 23))
    draw = ImageDraw.Draw(img)
    ox = oy = 0.0            # 先按第一帧的机器人位置定原点
    painted = 0
    depth_total = 0
    lidar = 0

    def to_px(x, y):
        return (x - ox) * ppm + side / 2.0, side / 2.0 - (y - oy) * ppm

    for stamp in times:
        depth = None
        for _t, m, _ in bag.read_messages(topics=[DEPTH],
                                          start_time=__import__('rospy').Time.from_sec(stamp)):
            depth = m
            break
        if depth is None:
            continue
        s = depth.header.stamp.to_sec()
        t_map_cam = tf.chain_to(depth_frame, s)
        if t_map_cam is None:
            print('第 %.1fs 处链断了，跳过' % (s - start))
            continue
        if painted == 0:
            ox, oy = t_map_cam[0, 3], t_map_cam[1, 3]
        print('  t=%5.1fs  相机在 map 里 (%+.2f, %+.2f, %+.2f)'
              % (s - start, t_map_cam[0, 3], t_map_cam[1, 3], t_map_cam[2, 3]))
        pts = depth_points(depth, k, stride=args.stride)
        world = pts.dot(t_map_cam[:3, :3].T) + t_map_cam[:3, 3]
        keep = world[(world[:, 2] > -0.05) & (world[:, 2] < 1.2)]
        depth_total += len(keep)
        for x, y, _z in keep:
            px, py = to_px(x, y)
            if 0 <= px < side and 0 <= py < side:
                draw.point((px, py), fill=(239, 68, 68))
        # 同一时刻的激光，画成蓝色 —— 和上面那帧深度是同一时刻，才有可比性
        scan = None
        for _t, m, _ in bag.read_messages(topics=[SCAN],
                                          start_time=__import__('rospy').Time.from_sec(s - 0.5)):
            scan = m
            break
        if scan is not None:
            t_map_laser = tf.chain_to('laser', scan.header.stamp.to_sec())
            if t_map_laser is not None:
                ang = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
                r = np.asarray(scan.ranges, dtype=np.float32)
                ok = np.isfinite(r) & (r > scan.range_min) & (r < 8.0)
                p = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]),
                              np.zeros(int(ok.sum()))], axis=1)
                for x, y, _z in p.dot(t_map_laser[:3, :3].T) + t_map_laser[:3, 3]:
                    px, py = to_px(x, y)
                    if 0 <= px < side and 0 <= py < side:
                        draw.point((px, py), fill=(56, 189, 248))
                        lidar += 1
        painted += 1

    draw.text((6, 6), 'red=depth(projected)  blue=lidar  frames=%d' % painted,
              fill=(210, 210, 210))
    img.save(args.out)
    print('投了 %d 帧，深度点 %d，激光点 %d → %s'
          % (painted, depth_total, lidar, args.out))
    bag.close()


if __name__ == '__main__':
    main()

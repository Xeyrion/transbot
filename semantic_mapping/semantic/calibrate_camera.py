#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用墙角（或任意两个互相垂直的竖直面）标定相机安装位姿。

    python3 calibrate_camera.py <bag> [--height 0.185] [--frames 20]

思路（全部闭式解，不用迭代优化）：
  1. 深度点云里找两个大平面（顺序 RANSAC），它们的法向在相机系里
  2. 两个法向的叉积就是"竖直方向"——把这个方向转到 +z，就得到俯仰和横滚
  3. 激光雷达看的是同样这两个面，拟出两条线
  4. 把深度面的法向和激光线的法向对齐 → 偏航
  5. 两个面的距离差 → 前后、左右两个平移
  6. 高度不参与求解，直接用尺子量的值（相机离地 - base_link 离地）

前提：那两个面必须是竖直的（靠墙立着就行），互相夹角在 60°~120° 之间。
"""
import argparse
import math
import os
import sys
import json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ros_runtime import ros_python, ros_python_major
if __name__ == '__main__' and sys.version_info[0] != ros_python_major():
    os.execvp(ros_python(), [ros_python(), os.path.abspath(__file__)]+sys.argv[1:])

import numpy as np
import rosbag

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pose
from extract_frames import depth_array as decode_depth
from camera_geometry import undistort,coefficients

DEPTH = '/camera/depth/image_raw'
DEPTH_INFO = '/camera/depth/camera_info'
RGB_INFO = '/camera/rgb/camera_info'
SCAN = '/scan'


def quat_matrix(q):
    return np.asarray(pose.quat_matrix(q))


def load_tf(bag):
    tf = {}
    for topic in ('/tf_static', '/tf'):
        for _t, msg, _ in bag.read_messages(topics=[topic]):
            for x in msg.transforms:
                key = (x.header.frame_id.strip('/'), x.child_frame_id.strip('/'))
                if key not in tf:
                    tr = x.transform
                    m = np.eye(4)
                    m[:3, :3] = quat_matrix((tr.rotation.x, tr.rotation.y,
                                             tr.rotation.z, tr.rotation.w))
                    m[:3, 3] = (tr.translation.x, tr.translation.y, tr.translation.z)
                    tf[key] = m
    return tf


def depth_array(msg):
    return decode_depth(msg)


def collect(bag, frames, step=4, rmax=6.0):
    """一次顺序遍历，收内参、深度点云和激光。

    传输中断的包没有索引，`read_messages(topics=...)` 会读不出东西；按话题名
    自己挑就绕开了索引。机器人静止，多帧叠起来能压噪声。
    """
    k = None
    infos={}; scans=[]; poses=[]; frame=None; first_stamp=None; last_stamp=None
    pts = []
    scan = None
    n = 0
    for topic, m, _ in bag.read_messages(topics=[RGB_INFO,DEPTH_INFO,SCAN,DEPTH,'/odom']):
        if topic in (RGB_INFO,DEPTH_INFO):
            infos[topic]=m
        elif topic == SCAN:
            scans.append(m)
        elif topic=='/odom':
            p,q=m.pose.pose.position,m.pose.pose.orientation
            poses.append((m.header.stamp.to_sec(),p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
        elif topic == DEPTH and n < frames:
            current=m.header.frame_id.strip('/')
            info=infos.get(RGB_INFO if 'color' in current or 'rgb' in current else DEPTH_INFO)
            if info is None:
                continue
            if frame is not None and frame!=current: raise ValueError('Depth optical frame changed during calibration')
            frame=current; k=np.array(info.K)
            if not np.isfinite(k).all() or min(k[0],k[4])<=0 or (info.width,info.height)!=(m.width,m.height):
                raise ValueError('Depth/intrinsics mismatch')
            coefficients(list(info.D),info.distortion_model or 'plumb_bob')
            stamp=m.header.stamp.to_sec()
            if last_stamp is not None and stamp<=last_stamp: raise ValueError('Ambiguous depth clock')
            first_stamp=stamp if first_stamp is None else first_stamp; last_stamp=stamp
            d = depth_array(m)
            h, w = d.shape
            v, u = np.mgrid[0:h:step, 0:w:step]
            z = d[0:h:step, 0:w:step]
            ok = (z > 0.3) & (z < rmax)
            zz = z[ok].astype(np.float64)
            xx,yy=undistort((u[ok]-k[2])/k[0],(v[ok]-k[5])/k[4],list(info.D))
            pts.append(np.stack([xx*zz,yy*zz,zz],axis=1))
            n += 1
        if n >= frames and scans and k is not None:
            break
    if not pts:
        sys.exit('没读到深度帧（包可能不完整）')
    relevant=[p for p in poses if first_stamp-.1<=p[0]<=last_stamp+.1]
    if len(relevant)<2: raise ValueError('Stationary calibration requires odometry evidence')
    start=relevant[0]
    if any(math.hypot(p[1]-start[1],p[2]-start[2])>.02 or abs(math.atan2(math.sin(p[3]-start[3]),math.cos(p[3]-start[3])))>.02 for p in relevant):
        raise ValueError('Robot moved during calibration; record stationary data')
    scan=min(scans,key=lambda m:abs(m.header.stamp.to_sec()-(first_stamp+last_stamp)/2))
    if abs(scan.header.stamp.to_sec()-(first_stamp+last_stamp)/2)>.15: raise ValueError('No time-compatible lidar scan')
    points=np.vstack(pts); points=points[np.isfinite(points).all(axis=1)]
    return points,k,scan,frame


def fit_plane(pts, iters=400, tol=0.02, seed=0):
    if len(pts)<3: raise ValueError('Not enough plane points')
    rng = np.random.RandomState(seed)
    best = None
    for _ in range(iters):
        i = rng.choice(len(pts), 3, replace=False)
        p0, p1, p2 = pts[i]
        n = np.cross(p1 - p0, p2 - p0)
        L = np.linalg.norm(n)
        if L < 1e-9:
            continue
        n = n / L
        inl = np.abs((pts - p0).dot(n)) < tol
        if best is None or inl.sum() > best[0]:
            best = (inl.sum(), inl)
    if best is None: raise ValueError('Degenerate plane points')
    cnt, inl = best
    sub = pts[inl]
    cen = sub.mean(axis=0)
    # 用 3x3 协方差的最小特征向量当法向 —— 对 26 万个点做 SVD 会把 Jetson 的内存吃光
    dev = sub - cen
    cov = dev.T.dot(dev)
    _, vecs = np.linalg.eigh(cov)
    n = vecs[:, 0]
    if n.dot(cen) > 0:            # 法向指向相机（原点）
        n = -n
    return n, cen, inl, cnt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--height', type=float, default=0.185,
                    help='相机光心离地高度（尺子量的）')
    ap.add_argument('--base-height', type=float, default=0.08,
                    help='base_link 离地高度（base_footprint→base_link 的 z）')
    ap.add_argument('--frames', type=int, default=20)
    ap.add_argument('--out',help='Write a candidate calibration JSON to a NEW file')
    ap.add_argument('--id',default='unvalidated_plane_candidate')
    ap.add_argument('--solve-yaw-xy',action='store_true',help='Explicit experimental lidar association, otherwise require measured fixed-yaw/fixed-xy')
    ap.add_argument('--fixed-yaw', type=float, default=None,
                    help='给定偏航（度），跳过激光配对。激光只看到一个面时配对不可靠，'
                         '这种模式下只解俯仰/横滚，偏航和平移沿用现有配置')
    ap.add_argument('--fixed-xy', default=None,
                    help='给定前后/左右（米，逗号分隔），配合 --fixed-yaw 用')
    args = ap.parse_args()
    if args.frames<4 or args.height<=args.base_height: ap.error('Need >=4 frames and measured camera height above base')
    if not args.solve_yaw_xy and (args.fixed_yaw is None or not args.fixed_xy):
        ap.error('Provide measured --fixed-yaw (degrees) and --fixed-xy (metres), or explicitly choose experimental --solve-yaw-xy')

    # 传输中断的包没有尾部索引；allow_unindexed 允许顺序读已落盘的部分
    bag = rosbag.Bag(args.bag, 'r')
    tf = load_tf(bag)
    pts, k, scan, optical_frame = collect(bag, args.frames)
    bag.close()
    print('深度点 %d 个（%d 帧）' % (len(pts), args.frames))

    # ---- 1. 两个大平面 ----
    remaining = np.arange(len(pts))
    planes = []
    for s in range(4):
        if len(remaining) < 1000:
            break
        n, cen, inl, cnt = fit_plane(pts[remaining], seed=s)
        if cnt < 800:
            break
        planes.append((n, cen, cnt))
        remaining = remaining[~inl]
    print('主平面：')
    for i, (n, cen, cnt) in enumerate(planes):
        print('  P%d %6d 点  法向 (%+.3f, %+.3f, %+.3f)  质心 z=%.2f'
              % (i, cnt, n[0], n[1], n[2], cen[2]))

    # 挑夹角 60~120° 的一对
    pair = None
    for i in range(len(planes)):
        for j in range(i + 1, len(planes)):
            a = math.degrees(math.acos(max(-1, min(1, abs(planes[i][0].dot(planes[j][0]))))))
            if 60 <= a <= 120 and (pair is None or
                                   min(planes[i][2], planes[j][2]) > min(planes[pair[0]][2], planes[pair[1]][2])):
                pair = (i, j, a)
    if pair is None:
        sys.exit('找不到夹角 60~120° 的两个大平面，换个位置重录')
    i, j, ang = pair
    n1, c1, _ = planes[i]
    n2, c2, _ = planes[j]
    print('\n选用 P%d 和 P%d，夹角 %.1f°' % (i, j, ang))

    # ---- 2. 俯仰/横滚：在"当前估计"上做增量修正 ----
    # 关键：不能拿光心系的原始法向直接算绝对旋转。光心帧本身带 -90° 偏航和 -90°
    # 横滚，绝对旋转会把这一层也吃进去，再乘一次 inv(T_link_optical) 就重复了
    # （表现为固定偏航 -2.87° 却报出 80.9°）。
    def rot_to_z(v):
        v = v / np.linalg.norm(v)
        if v[2] < 0:
            v = -v
        ax = np.cross(v, [0.0, 0.0, 1.0])
        s_ = np.linalg.norm(ax)
        c_ = v.dot([0.0, 0.0, 1.0])
        if s_ < 1e-9:
            return np.eye(3)
        ax = ax / s_
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        return np.eye(3) + s_ * K + (1 - c_) * K.dot(K)

    index=pose.Tf()
    for key,matrix in tf.items(): index.add(key[0],key[1],0.,matrix.tolist(),static=True)
    index.finish(); optical=index.chain_to(optical_frame,0.,root='camera_link')
    if optical is None: raise ValueError('Missing camera_link to actual depth optical frame')
    link_opt=np.asarray(optical); R_link_opt=link_opt[:3,:3]

    def level(R_current):
        h1_ = R_current.dot(n1)
        h2_ = R_current.dot(n2)
        R_corr = rot_to_z(np.cross(h1_, h2_))
        return R_corr.dot(R_current), R_corr.dot(h1_), R_corr.dot(h2_)

    # 以配置值作为起点，只修俯仰/横滚
    yaw_cfg = -0.05
    R_start = np.array([[math.cos(yaw_cfg), -math.sin(yaw_cfg), 0],
                        [math.sin(yaw_cfg), math.cos(yaw_cfg), 0], [0, 0, 1]]).dot(R_link_opt)
    R_tilt, h1, h2 = level(R_start)
    print('校正后两面法向的 z 分量：%+.3f / %+.3f（应接近 0）' % (h1[2], h2[2]))

    if args.fixed_yaw is not None:
        yaw0 = math.radians(args.fixed_yaw)
        Rz = np.array([[math.cos(yaw0), -math.sin(yaw0), 0],
                       [math.sin(yaw0), math.cos(yaw0), 0], [0, 0, 1]])
        R = Rz.dot(R_link_opt)
        R, _, _ = level(R)          # 只叠加俯仰/横滚修正
        x0, y0 = (0.08, 0.0)
        if args.fixed_xy:
            x0, y0 = [float(v) for v in args.fixed_xy.split(',')]
        t = np.array([x0, y0, args.height - args.base_height])
        print('\n[固定模式] 偏航 %.2f°，平移 (%.3f, %.3f, %.3f)；只解俯仰/横滚'
              % (args.fixed_yaw, t[0], t[1], t[2]))
        report(R,t,link_opt,args)
        return

    # ---- 3. 激光：两条线 ----
    ang_all = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    r = np.asarray(scan.ranges, dtype=np.float32)
    ok = np.isfinite(r) & (r > scan.range_min) & (r < 6.0)
    laser = np.stack([r[ok] * np.cos(ang_all[ok]), r[ok] * np.sin(ang_all[ok])], axis=1)
    t_base_laser = tf.get(('base_link', 'laser'))
    if t_base_laser is None:
        sys.exit('包里没有 base_link→laser')
    lp = laser.dot(t_base_laser[:2, :2].T) + t_base_laser[:2, 3]

    def fit_line2d(p, iters=300, tol=0.03, seed=0):
        rng = np.random.RandomState(seed)
        best = None
        for _ in range(iters):
            a, b = p[rng.choice(len(p), 2, replace=False)]
            d = b - a
            L = np.linalg.norm(d)
            if L < 1e-6:
                continue
            nn = np.array([-d[1], d[0]]) / L
            inl = np.abs((p - a).dot(nn)) < tol
            if best is None or inl.sum() > best[0]:
                best = (inl.sum(), inl, nn, a)
        cnt, inl, nn, a = best
        sub = p[inl]
        cen = sub.mean(axis=0)
        dev = sub - cen
        _, vecs = np.linalg.eigh(dev.T.dot(dev))
        nn = vecs[:, 0]
        if nn.dot(cen) > 0:
            nn = -nn
        return nn, cen, cnt

    rem = np.arange(len(lp))
    lines = []
    for s in range(3):
        if len(rem) < 30:
            break
        nn, cen, cnt = fit_line2d(lp[rem], seed=s)
        if cnt < 25:
            break
        lines.append((nn, cen, cnt))
        rem = rem[~(np.abs((lp[rem] - cen).dot(nn)) < 0.03)]
    print('激光线 %d 条' % len(lines))
    for i2, (nn, cen, cnt) in enumerate(lines):
        print('  L%d %3d 点  法向 (%+.3f, %+.3f)  朝向 %+.1f°'
              % (i2, cnt, nn[0], nn[1], math.degrees(math.atan2(nn[1], nn[0]))))

    # ---- 4. 配对：按方向最接近 ----
    best = None
    for a_ in range(len(lines)):
        for b_ in range(len(lines)):
            if a_ == b_:
                continue
            # 深度面 1 ↔ 线 a_，深度面 2 ↔ 线 b_
            e = abs(h1[0] * lines[a_][0][0] + h1[1] * lines[a_][0][1]) + \
                abs(h2[0] * lines[b_][0][0] + h2[1] * lines[b_][0][1])
            score = 2 - e
            if best is None or score < best[0]:
                best = (score, a_, b_)
    if best is None:
        sys.exit('激光线和深度面对不上')
    _, la, lb = best
    g1, gc1, _ = lines[la]
    g2, gc2, _ = lines[lb]
    print('配对：深度面1 ↔ 激光线%d，深度面2 ↔ 激光线%d' % (la, lb))

    # ---- 5. 偏航 ----
    a1 = math.atan2(h1[1], h1[0])
    b1 = math.atan2(g1[1], g1[0])
    a2 = math.atan2(h2[1], h2[0])
    b2 = math.atan2(g2[1], g2[0])

    def wrap(x):
        while x > math.pi:
            x -= 2 * math.pi
        while x < -math.pi:
            x += 2 * math.pi
        return x

    da,db=wrap(b1-a1),wrap(b2-a2)
    if abs(wrap(da-db))>math.radians(10): raise ValueError('The two plane/line yaw estimates disagree')
    dyaw = math.atan2(math.sin(da)+math.sin(db),math.cos(da)+math.cos(db))
    Rz = np.array([[math.cos(dyaw), -math.sin(dyaw), 0],
                   [math.sin(dyaw), math.cos(dyaw), 0], [0, 0, 1]])
    R, _, _ = level(Rz.dot(R_link_opt))
    print('偏航修正 %+.2f°' % math.degrees(dyaw))

    # ---- 6. 平移 x/y：两个面的距离方程 ----
    # 面 i 过点 R·c_i + t，法向 h_i（转到 base 后）；激光线 i 法向 g_i、过 gc_i
    # 方程：h_i · (R c_i + t) = g_i · gc_i
    A = []
    bvec = []
    for (h, c, g, gc) in ((h1, c1, g1, gc1), (h2, c2, g2, gc2)):
        hh = Rz.dot(h)
        A.append([hh[0], hh[1]])
        bvec.append(g.dot(gc) - hh.dot(R.dot(c)))
    if np.linalg.cond(np.array(A))>30: raise ValueError('Ill-conditioned plane/line association')
    sol = np.linalg.solve(np.array(A), np.array(bvec))
    tz = args.height - args.base_height
    t = np.array([sol[0], sol[1], tz])
    print('平移 (%.3f, %.3f, %.3f)' % tuple(t))

    report(R,t,link_opt,args)


def report(R,t,t_link_opt,args):
    # ---- 换算成 base_link ← camera_link ----
    # 相机链 camera_link → depth_frame → optical
    T_base_opt = np.eye(4)
    T_base_opt[:3, :3] = R
    T_base_opt[:3, 3] = t
    T_base_link = T_base_opt.dot(np.linalg.inv(t_link_opt))

    yaw = math.atan2(T_base_link[1, 0], T_base_link[0, 0])
    pitch = math.asin(max(-1, min(1, -T_base_link[2, 0])))
    roll = math.atan2(T_base_link[2, 1], T_base_link[2, 2])
    x, y, z = T_base_link[:3, 3]
    print('\n==== base_link → camera_link ====')
    print('  x=%.4f  y=%.4f  z=%.4f' % (x, y, z))
    print('  yaw=%+.2f°  pitch=%+.2f°  roll=%+.2f°'
          % (math.degrees(yaw), math.degrees(pitch), math.degrees(roll)))
    print('\nlaunch 里写成（单位：米/弧度）：')
    print('  args="%.4f %.4f %.4f %.4f %.4f %.4f base_link camera_link 30"'
          % (x, y, z, yaw, pitch, roll))
    if args.out:
        data=dict(schema_version=1,id=args.id,estimated=True,validation='candidate_requires_independent_measurement',
            source_bag=os.path.abspath(args.bag),method='stationary_two_plane',
            measured_anchors=dict(height=args.height,base_height=args.base_height,fixed_yaw=args.fixed_yaw,fixed_xy=args.fixed_xy),
            transforms=[dict(parent='base_link',child='camera_link',xyz=[float(x),float(y),float(z)],
                quaternion_xyzw=[float(v) for v in pose.matrix_quat(T_base_link.tolist())])])
        fd=os.open(args.out,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o644)
        with os.fdopen(fd,'w') as f: json.dump(data,f,indent=2)


if __name__ == '__main__':
    main()

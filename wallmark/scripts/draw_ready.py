#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# draw_ready.py -- 输入车体到画板距离，自动摆位 + 定向画线
# 用法示例:
#   source ../../console/env.sh
#   python2 draw_ready.py --dist 40 --direction down --length 1.0        # 40cm是 base_link 到板
#   python2 draw_ready.py --dist 40 --direction up --length 1.0 --dry    # 只算不执行
#
# 流程: 激光核对距离 -> 2R逆解算(j7,j8) -> 慢速摆位到预碰点(离板5mm)
#       -> 人工确认触碰(微调) -> 按方向画线 -> 法向回退
import math, time, argparse
import rospy
from sensor_msgs.msg import LaserScan
from transbot_msgs.msg import Arm, Joint
from transbot_msgs.srv import RobotArm

L1, L2 = 10.0, 24.0          # cm, l2已含笔长(j9=159夹持)；实测值
J7_OFF, J8_OFF = 95.0, 180.0 # 标定: e1=j7-95, rel=j8-180, e2=j7+j8-275
J7_LIM, J8_LIM = (0.0, 225.0), (15.0, 240.0)  # 与最新 GUI 一致
LASER_X, ARM_X = 0.0, 12.9  # cm, 相对base_link (LASER_X=0: 2026-10-20定, wallmark验证够用)
D2R = math.pi / 180.0

def fk(j7, j8):
    e1 = (j7 - J7_OFF) * D2R
    e2 = (e1 / D2R + (j8 - J8_OFF)) * D2R
    return (L1 * math.cos(e1) + L2 * math.cos(e2),
            L1 * math.sin(e1) + L2 * math.sin(e2))

def ik(x, z):
    D = math.hypot(x, z)
    D = max(abs(L2 - L1) + 1.0, min(L1 + L2 - 2.0, D))  # 留1~2cm躲折叠/伸直奇异
    c = max(-1.0, min(1.0, (D * D - L1 * L1 - L2 * L2) / (2 * L1 * L2)))
    phi = math.atan2(z, x)
    sols = []
    for sgn in (1.0, -1.0):
        beta = sgn * math.acos(c)
        e1 = phi - math.atan2(L2 * math.sin(beta), L1 + L2 * math.cos(beta))
        j7 = e1 / D2R + J7_OFF
        j8 = beta / D2R + J8_OFF
        if J7_LIM[0] <= j7 <= J7_LIM[1] and J8_LIM[0] <= j8 <= J8_LIM[1]:
            sols.append((j7, j8, abs(beta / D2R)))
    return sols

def read_pose():
    rospy.wait_for_service('/CurrentAngle', timeout=5)
    res = rospy.ServiceProxy('/CurrentAngle', RobotArm)(apply='')
    return {j.id: float(j.angle) for j in res.RobotArm.joint}

_pub = None
def send(j7=None, j8=None, run_time=800):
    global _pub
    if _pub is None:
        _pub = rospy.Publisher('/TargetAngle', Arm, queue_size=10)
        time.sleep(0.3)
    a = Arm()
    if j7 is not None:
        a.joint.append(Joint(id=7, run_time=int(run_time), angle=float(j7)))
    if j8 is not None:
        a.joint.append(Joint(id=8, run_time=int(run_time), angle=float(j8)))
    _pub.publish(a)

def goto(j7, j8, step=3.0):
    cur = read_pose()
    n = int(max(abs(j7 - cur[7]), abs(j8 - cur[8])) / step) + 1
    for k in range(1, n + 1):
        send(cur[7] + (j7 - cur[7]) * k / n, cur[8] + (j8 - cur[8]) * k / n)
        time.sleep(1.2)
    time.sleep(1.0)
    print('  reached:', read_pose())

def forward_laser():
    for _ in range(3):
        try:
            m = rospy.wait_for_message('/scan', LaserScan, timeout=30)
            vals = []
            for i, r in enumerate(m.ranges):
                deg = (m.angle_min + i * m.angle_increment) / D2R
                d = abs((deg - 180 + 180) % 360 - 180)  # laser系180°=车头(雷达倒装)
                if d < 6 and 0.15 < r < 12:
                    vals.append(r)
            if vals:
                vals.sort()
                return sum(vals) / len(vals) * 100.0  # cm, laser到板
        except rospy.ROSException:
            print('  scan wait timeout, retry...')
    return None

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, required=True, help='车体中心到画板距离(cm)')
    ap.add_argument('--z', type=float, default=8.0, help='触笔点相对臂座高度(cm)')
    ap.add_argument('--direction', choices=['down', 'up'], default='down')
    ap.add_argument('--length', type=float, default=1.0, help='线长(cm)')
    ap.add_argument('--dry', action='store_true')
    args = ap.parse_args()
    rospy.init_node('draw_ready', anonymous=True)

    # 1. 激光核对
    print('[1] laser check...')
    meas = forward_laser()
    if meas is not None:
        print('  laser->board %.1fcm => base->board %.1fcm (输入%.1f)'
              % (meas, meas + LASER_X, args.dist))
        if abs((meas + LASER_X) - args.dist) > 3.0:
            print('  WARN: 实测与输入差>3cm, 请确认车没动/输入单位是cm')
    else:
        print('  WARN: 读不到scan, 跳过核对')

    # 2. 逆解 (预碰点: 板前5mm)
    x = args.dist - ARM_X - 0.5
    sols = ik(x, args.z)
    print('[2] IK target x=%.1f z=%.1f => %d feasible' % (x, args.z, len(sols)))
    if not sols:
        print('  NO SOLUTION: 距离太近(<折叠)或太远(>臂展), 请调车位'); raise SystemExit(1)
    cur = read_pose()
    sols.sort(key=lambda s: abs(s[0] - cur[7]) + abs(s[1] - cur[8]))
    j7t, j8t, bend = sols[0]
    print('  choose j7=%.1f j8=%.1f (肘弯%.0f°, 当前%s)' % (j7t, j8t, bend, cur))
    if bend > 120:
        print('  WARN: 肘部折叠>120°,  although可控但注意连杆/相机干涉')
    if args.dry:
        print('DRY RUN DONE'); raise SystemExit(0)

    # 3. 摆位
    print('[3] goto pre-touch...')
    goto(j7t, j8t)
    raw_input('人工确认: 点动到笔尖刚碰板(±0.5°微调), 回车开画(Ctrl-C取消)...')
    c = read_pose()
    print('  contact:', c)

    # 4. 画线: 雅可比保法向(dx=0), 步进0.25cm
    print('[4] draw %s %.1fcm...' % (args.direction, args.length))
    e1 = (c[7] - J7_OFF) * D2R
    e2 = (e1 / D2R + (c[8] - J8_OFF)) * D2R
    det = L1 * L2 * math.sin(e2 - e1)
    steps = int(round(args.length / 0.25))
    sgn = -1.0 if args.direction == 'down' else 1.0
    for k in range(steps):
        if abs(det) > 30:  # 非奇异: 双关节协调, 法向dx=0, 竖直dz=sgn*0.25
            dz = sgn * 0.25
            a11, a12 = -L1 * math.sin(e1), -L2 * math.sin(e2)
            a21, a22 = L1 * math.cos(e1), L2 * math.cos(e2)
            de1 = (0 * a22 - a12 * dz) / det
            de2 = (a11 * dz - 0 * a21) / det
            dj7 = de1 / D2R
            dj8 = (de2 - de1) / D2R
        else:  # 近奇异: j8单动
            dj7, dj8 = 0.0, sgn * 0.25 / (0.419 * math.cos(e2))
        c = read_pose()
        nj7 = max(J7_LIM[0], min(J7_LIM[1], c[7] + dj7))
        nj8 = max(J8_LIM[0], min(J8_LIM[1], c[8] + dj8))
        print('  step %d: j7->%.1f j8->%.1f' % (k + 1, nj7, nj8))
        send(nj7, nj8, run_time=1000)
        time.sleep(2.0)
        e1 = (nj7 - J7_OFF) * D2R
        e2 = (e1 / D2R + (nj8 - J8_OFF)) * D2R
        det = L1 * L2 * math.sin(e2 - e1)

    # 5. 法向回退1cm
    print('[5] retract...')
    c = read_pose()
    send(c[7] + 2.0, c[8], run_time=800)
    time.sleep(2.0)
    print('DONE. 退回0.8m拍照差分验痕')

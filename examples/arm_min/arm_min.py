#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Transbot 机械臂单关节控制示例（带限位与角度读回）。

用法:
    source ../../env.sh
    python3 arm_min.py [id] [angle] [run_time]

参数:
    id: 7=底座, 8=升降, 9=夹爪
    angle: 绝对角度（度）
    run_time: 运动时长（毫秒），必须大于 0
"""
import sys
import time

import rospy
from transbot_msgs.msg import Arm, Joint
from transbot_msgs.srv import RobotArm

# 控制端统一限位（与 transbot_gui.py 一致）：
# id7 0~225，id8 15~240，id9 60~180。
# 旧手册的 id8 30~270 不再采用；GUI/URDF/真机 agent 都用这一组。
LIMITS = {7: (0, 225), 8: (15, 240), 9: (60, 180)}


def main():
    jid = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    angle = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
    run_time = int(sys.argv[3]) if len(sys.argv) > 3 else 800

    if jid not in LIMITS:
        raise SystemExit('id 必须为 7/8/9')
    lo, hi = LIMITS[jid]
    if not lo <= angle <= hi:
        raise SystemExit('角度 %.1f 超出范围 %d~%d' % (angle, lo, hi))
    if run_time <= 0:
        raise SystemExit('run_time 必须大于 0（驱动会丢弃 0）')

    rospy.init_node('arm_min', anonymous=True)
    pub = rospy.Publisher('/TargetAngle', Arm, queue_size=10)

    # 等待 /TargetAngle 订阅者，避免首包丢失；超时 5 秒。
    deadline = time.time() + 5.0
    while (pub.get_num_connections() == 0
           and time.time() < deadline
           and not rospy.is_shutdown()):
        rospy.sleep(0.05)
    if pub.get_num_connections() == 0:
        raise SystemExit('无 /TargetAngle 订阅者：请确认 transbot_node 已启动')

    pub.publish(Arm(joint=[Joint(id=jid, run_time=run_time, angle=angle)]))
    print('sent: id=%d angle=%.1f run_time=%dms' % (jid, angle, run_time))

    # publish 为异步调用；等待运动完成后再查询角度。
    rospy.sleep(run_time / 1000.0 + 1.0)

    try:
        rospy.wait_for_service('/CurrentAngle', timeout=3.0)
        resp = rospy.ServiceProxy('/CurrentAngle', RobotArm)(apply='')
        for j in resp.RobotArm.joint:
            print('readback: id%d=%.1f' % (j.id, j.angle))
    except Exception as e:
        # 读回失败不影响已发布的控制指令。
        print('readback failed: %s' % e)


if __name__ == '__main__':
    main()

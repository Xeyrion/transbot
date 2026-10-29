#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Transbot 机械臂单关节控制示例（最小实现）。

用法: python3 arm_simplest.py [id] [angle] [run_time]
      id=7/8/9（底座/升降/夹爪），angle=绝对角度，run_time=毫秒且不可为 0
示例: python3 arm_simplest.py 7 100 800
说明: 仅演示 /TargetAngle 发布，不含限位检查、连接确认和角度读回。
"""
import sys
import rospy
from transbot_msgs.msg import Arm, Joint

jid = int(sys.argv[1]) if len(sys.argv) > 1 else 7
angle = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
run_time = int(sys.argv[3]) if len(sys.argv) > 3 else 800

rospy.init_node('arm_simplest', anonymous=True)
pub = rospy.Publisher('/TargetAngle', Arm, queue_size=10)

# 固定延时无法确认订阅关系；高延迟链路应轮询 get_num_connections()。
rospy.sleep(1.0)

pub.publish(Arm(joint=[Joint(id=jid, run_time=run_time, angle=angle)]))

# publish 为异步调用；等待消息发送及运动完成后再退出。
rospy.sleep(run_time / 1000.0 + 1.0)
print('published: id=%d angle=%.1f run_time=%dms' % (jid, angle, run_time))

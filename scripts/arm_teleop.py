#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Keyboard teleop: left hand = arm+gimbal, right hand = base (teleop style)."""
import sys, tty, termios, select, time
import rospy
from geometry_msgs.msg import Twist
from transbot_msgs.msg import Arm, Joint, PWMServo
from transbot_msgs.srv import RobotArm, RobotArmRequest

# ---------- 机械臂关节(舵机 id)，与 gui/app.py ARM_LIMITS / URDF 保持一致 ----------
LIMITS = {
    'j1': (0,   225),   # id 7  关节1(底座摆动)，URDF arm_joint1
    'j2': (15,  240),   # id 8  关节2(升降)，URDF arm_joint2
    'gp': (60,  180),   # id 9  夹爪
    'pt': (60,  120),   # id 1  云台水平
}
HOME      = {'j1': 90, 'j2': 90, 'gp': 120, 'pt': 90}
ARM_STEPS = [1, 5, 10, 20]

# ---------- 底盘（与驱动硬限幅 / GUI LIN_MAX 一致，勿超 0.4） ----------
LINEAR_MAX, ANGULAR_MAX = 0.4, 2.0
SPEED_FACTOR = [0.2, 0.35, 0.5, 0.7, 1.0]

ARM_KEYS = {
    'q': ('j1', +1), 'z': ('j1', -1),
    'w': ('j2', +1), 'x': ('j2', -1),
    'e': ('gp', +1), 'c': ('gp', -1),
    'r': ('pt', +1), 'v': ('pt', -1),
}
MOVE_KEYS = {
    'i': ( 1.0,  0.0),   # 前进
    ',': (-1.0,  0.0),   # 后退
    'j': ( 0.0,  1.0),   # 左转
    'l': ( 0.0, -1.0),   # 右转
    'k': ( 0.0,  0.0),   # 停
}

state = {'j1': 90, 'j2': 90, 'gp': 120, 'pt': 90}
arm_step_idx = 3
speed_idx    = 2
twist = Twist()
last_cmd = 0.0
pub_arm, pub_ptz, pub_vel = None, None, None

def clamp(name, v):
    lo, hi = LIMITS[name]
    return max(lo, min(hi, v))

def fetch_current():
    try:
        rospy.wait_for_service('/CurrentAngle', timeout=3)
        res = rospy.ServiceProxy('/CurrentAngle', RobotArm)(RobotArmRequest('getJoint'))
        for j in res.RobotArm.joint:
            if   j.id == 7: state['j1'] = int(j.angle)
            elif j.id == 8: state['j2'] = int(j.angle)
            elif j.id == 9: state['gp'] = int(j.angle)
    except Exception as e:
        print("(查询当前角度失败, 用默认值: %s)" % e)

def move_arm(j1, j2, gp, run_time):
    m = Arm()
    for id, a in ((7, j1), (8, j2), (9, gp)):
        jt = Joint(); jt.id = id; jt.angle = int(a); jt.run_time = run_time
        m.joint.append(jt)
    pub_arm.publish(m)

def move_ptz(angle):
    p = PWMServo(); p.id = 1; p.angle = int(angle)
    pub_ptz.publish(p)

def do_axis(name, sign, run_time):
    state[name] = clamp(name, state[name] + sign * ARM_STEPS[arm_step_idx])
    if name == 'pt': move_ptz(state[name])
    else:            move_arm(state['j1'], state['j2'], state['gp'], run_time)

def go_home():
    for k, v in HOME.items(): state[k] = v
    move_arm(state['j1'], state['j2'], state['gp'], 1200)
    move_ptz(state['pt'])

def brake():
    twist.linear.x = twist.angular.z = 0.0
    pub_vel.publish(twist)

def help_text():
    print("""
==================================== Transbot 遥控 ====================================
  机械臂(左手)                        底盘移动(右手, teleop风格)
  q / z    关节1  摆动  + / -         i          前进
  w / x    关节2  升降  + / -         ,          后退
  e / c    夹爪    夹紧 / 松开         j          左转
  r / v    云台    右 / 左             l          右转
  a / d    步长    %s / %s               k          停车
  s        回到初始姿态                 u / o      减速 / 加速
  f        打印状态
======================================================================================
  Esc / Ctrl+C  退出 (机械臂保持当前角度, 底盘立即停车)
""".lstrip() % (ARM_STEPS[arm_step_idx], ARM_STEPS[min(arm_step_idx+1, len(ARM_STEPS)-1)]))

def status():
    sys.stdout.write("\r  机械臂 J1=%3d J2=%3d 夹爪=%3d 云台=%3d | 步长%2d° | 底盘 x=%+.2f z=%+.2f | 速度x%.2f" %
                     (state['j1'], state['j2'], state['gp'], state['pt'],
                      ARM_STEPS[arm_step_idx], twist.linear.x, twist.angular.z,
                      SPEED_FACTOR[speed_idx]))
    sys.stdout.flush()

def main():
    global pub_arm, pub_ptz, pub_vel, arm_step_idx, speed_idx
    rospy.init_node('arm_teleop_keyboard')
    pub_arm = rospy.Publisher('/TargetAngle', Arm, queue_size=10)
    pub_ptz = rospy.Publisher('/PWMServo',  PWMServo, queue_size=10)
    pub_vel = rospy.Publisher('/cmd_vel',   Twist, queue_size=10)
    fetch_current()
    help_text()
    print("已连接, 当前状态:"); status(); print()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key in ('\x03', '\x1b', '\x7f'):
                    brake(); break
                elif key in ARM_KEYS:
                    axis, sign = ARM_KEYS[key]
                    do_axis(axis, sign, 300)
                elif key in MOVE_KEYS:
                    lx, az = MOVE_KEYS[key]
                    f = SPEED_FACTOR[speed_idx]
                    twist.linear.x  = lx * LINEAR_MAX * f
                    twist.angular.z = az * ANGULAR_MAX * f
                    pub_vel.publish(twist)
                    last_cmd = time.time()
                elif key == 'a': arm_step_idx = max(0, arm_step_idx - 1)
                elif key == 'd': arm_step_idx = min(len(ARM_STEPS) - 1, arm_step_idx + 1)
                elif key == 'u': speed_idx = max(0, speed_idx - 1); brake()
                elif key == 'o': speed_idx = min(len(SPEED_FACTOR) - 1, speed_idx + 1); brake()
                elif key == 's': go_home()
                elif key == 'f': status()
                else:            brake(); last_cmd = time.time()
                status()
            else:
                # 松开移动键 0.5s 后自动停车, 防溜车
                if twist.linear.x != 0.0 or twist.angular.z != 0.0:
                    if time.time() - last_cmd > 0.5:
                        brake(); status()
        brake()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n退出. 机械臂保持在当前角度, 底盘已停车.")

if __name__ == '__main__':
    main()
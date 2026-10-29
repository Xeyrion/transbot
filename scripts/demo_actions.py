#!/usr/bin/env python3
"""One-shot demo: buzzer -> RGB effects -> gimbal sweep -> arm grasp -> home.
Safe for showroom use: the mobile base never moves.
Requires: transbot_msgs (vendor), robot online. Run after sourcing ../env.sh.
"""
import rospy
import time
from transbot_msgs.srv import RGBLight, Buzzer
from transbot_msgs.msg import PWMServo, Arm, Joint

rospy.init_node('demo_actions', anonymous=True)

def srv(name, T, **kw):
    try:
        rospy.wait_for_service(name, timeout=5)
        c = rospy.ServiceProxy(name, T)
        r = c(**kw)
        print(name, "->", r)
        return r
    except Exception as e:
        print("srv fail", name, e)

pub_servo = rospy.Publisher('/PWMServo', PWMServo, queue_size=10)
pub_arm = rospy.Publisher('/TargetAngle', Arm, queue_size=10)

def servo(id, ang, wait=0.8):
    pub_servo.publish(PWMServo(id=id, angle=int(ang)))
    time.sleep(wait)

def arm(joints, wait=1.5):
    a = Arm()
    for jid, ang, rt in joints:
        a.joint.append(Joint(id=jid, run_time=rt, angle=float(ang)))
    pub_arm.publish(a)
    time.sleep(wait)

print("== 1. buzzer x2")
srv('/Buzzer', Buzzer, buzzer=200)
time.sleep(0.3)
srv('/Buzzer', Buzzer, buzzer=200)

print("== 2. RGB effects: flowing / breathing / battery")
for ef, t in [(1, 2), (3, 2), (6, 2)]:
    srv('/RGBLight', RGBLight, effect=ef, speed=3)
    time.sleep(t)
srv('/RGBLight', RGBLight, effect=0, speed=1)

print("== 3. gimbal sweep (yaw id1)")
servo(1, 60); servo(1, 90); servo(1, 120); servo(1, 90)

print("== 4. arm: rotate -> lift -> open/close claw -> home")
# All angles within URDF limits (cf. transbot_gui.py ARM_LIMITS):
# base id7 0-225° (90, 191), lift id8 15-240° (100, 50), gripper id9 60-180° (150, 70, 90).
arm([(7, 90, 600), (8, 100, 700), (9, 150, 400)], wait=1.6)
arm([(9, 70, 400)], wait=1.2)
arm([(9, 150, 400)], wait=1.0)
arm([(8, 50, 700), (7, 191, 600), (9, 90, 400)], wait=2.0)

srv('/Buzzer', Buzzer, buzzer=100)
print("DONE")
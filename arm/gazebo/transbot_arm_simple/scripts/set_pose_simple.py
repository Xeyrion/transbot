#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Publish one j7/j8 pose to the simple Gazebo arm position controller.

Usage:
    rosrun transbot_arm_simple set_pose_simple.py 169.6 105.4
    rosrun transbot_arm_simple set_pose_simple.py --j7 95 --j8 180

j7/j8 are degrees.  The node converts them to the joint convention used
by arm_simple.urdf (fixed zeros, no calibration file):

    arm_joint1 = radians(j7 - 95)
    arm_joint2 = radians(j8 - 180)
"""
from __future__ import print_function

import argparse
import math
import sys
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

STEP = 0.05          # rad per command (~2.9 deg)
PERIOD = 0.05        # s between commands


def wrap_pi(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def main():
    ap = argparse.ArgumentParser(description='set simple Gazebo arm pose (j7/j8 deg)')
    ap.add_argument('j7_pos', nargs='?', type=float, default=None)
    ap.add_argument('j8_pos', nargs='?', type=float, default=None)
    ap.add_argument('--j7', dest='j7_opt', type=float, default=None)
    ap.add_argument('--j8', dest='j8_opt', type=float, default=None)
    ap.add_argument('--wait', type=float, default=8.0,
                    help='max seconds to wait for a controller subscriber')
    # rospy/roslaunch append __name:=... __log:=...; argparse must ignore them.
    args = ap.parse_args(rospy.myargv(argv=sys.argv)[1:])

    j7 = args.j7_opt if args.j7_opt is not None else args.j7_pos
    j8 = args.j8_opt if args.j8_opt is not None else args.j8_pos
    if j7 is None:
        j7 = 169.6
    if j8 is None:
        j8 = 105.4

    rospy.init_node('set_simple_arm_pose', anonymous=True)
    pub = rospy.Publisher('/arm_controller/command', Float64MultiArray,
                          queue_size=1)

    # Use wall-clock time here: with use_sim_time the node may start before the
    # first /clock message, in which case rospy.Time.now() is still 0 and a
    # sim-time deadline would already be in the past.
    deadline = time.time() + args.wait
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if time.time() > deadline:
            rospy.logwarn('no /arm_controller/command subscriber after %.1fs'
                          % args.wait)
            return 1
        time.sleep(0.1)

    t1 = wrap_pi(math.radians(j7 - 95.0))
    t2 = wrap_pi(math.radians(j8 - 180.0))

    # Start from the measured pose and ramp to the target in small steps.
    q1, q2 = 0.0, 0.0
    try:
        js = rospy.wait_for_message('/joint_states', JointState, timeout=3.0)
        idx = dict((n, i) for i, n in enumerate(js.name))
        q1 = wrap_pi(js.position[idx['arm_joint1']])
        q2 = wrap_pi(js.position[idx['arm_joint2']])
    except Exception as exc:
        rospy.logwarn('could not read /joint_states (%s); starting at 0' % exc)

    n = max(1, int(math.ceil(max(abs(t1 - q1), abs(t2 - q2)) / STEP)))
    for k in range(1, n + 1):
        a = q1 + (t1 - q1) * k / n
        b = q2 + (t2 - q2) * k / n
        pub.publish(Float64MultiArray(data=[a, b]))
        time.sleep(PERIOD)

    rospy.loginfo('j7=%.2f j8=%.2f -> t1=%.4f t2=%.4f rad (%d steps)'
                  % (j7, j8, t1, t2, n))
    time.sleep(0.2)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

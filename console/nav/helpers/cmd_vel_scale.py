#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Optional /cmd_vel gain corrector, for the chassis firmware's angular error.

What the 2026-11-02 on-site check found
---------------------------------------
`transbot_driver.py` does not do kinematics. It packs the command straight into
the serial frame:

    velocity_params = struct.pack('h', int(velocity * 100))
    angular_params  = struct.pack('h', int(angular  * 100))

and `get_motion_data()` returns whatever the MCU reports back. So both angular
errors live in the chassis firmware, not in ROS:

  * command side:  cmd 0.30 rad/s -> actual ~0.42-0.44 rad/s  (1.41-1.46x)
  * feedback side: actual -> MCU reports 0.50x  (fixed by angular_scale=2.0
                   in base_node, which is the right place for that one)

There is no ROS parameter that scales the outgoing command. The driver's
dynamic_reconfigure only clamps (linear_speed_limit / angular_speed_limit), it
does not scale. The vendor's calibrate_angular.py is the stock Pi Robot script
and is closed-loop on odom, so it cannot see this error either.

Two ways to deal with it
------------------------
1. Do nothing and compensate in the consumer: divide every wz you publish by
   1.41. Fine for hand-written test scripts, fragile for move_base, because DWA
   assumes its command is executed as commanded -- a 1.41x gain makes it
   oscillate near goals.
2. Insert this node. It is opt-in and changes nothing unless you wire it in:

       <node pkg="transbot_bringup" type="transbot_driver.py"
             name="transbot_node">
           <remap from="/cmd_vel" to="/cmd_vel_driver"/>
       </node>

       python3 cmd_vel_scale.py --angular-scale 0.709

   Then everything publishes /cmd_vel as usual and the driver sees
   /cmd_vel_driver.

Why 0.709: 1/1.41. Re-measure with angular_gain_calibration.py after charging
the battery -- the gain moves with pack voltage (open-loop PWM), so 1.41 at
11.2 V is not 1.41 at 12.2 V.

Safety
------
This node has its own deadman: if the input goes quiet for --timeout seconds
while the last command was non-zero, it publishes zero. Without that, adding a
node in front of the driver creates a new way for the robot to keep driving
after a crash.
"""
import argparse
import math
import sys
import time

import rospy
from geometry_msgs.msg import Twist

ZERO = Twist()


class Scaler(object):
    def __init__(self, in_topic, out_topic, linear_scale, angular_scale,
                 timeout, dry_run=False, teleop_topic=None, teleop_hold=0.3,
                 teleop_scale=1.0):
        self.linear_scale = linear_scale
        self.angular_scale = angular_scale
        self.angular_deadzone = 0.0
        self.timeout = timeout
        self.dry_run = dry_run
        self.last_rx = time.time()
        self.last_nonzero = False
        self.fired = False
        self.pub = rospy.Publisher(out_topic, Twist, queue_size=1)
        rospy.Subscriber(in_topic, Twist, self._cb, queue_size=10)
        # Optional second input: hand driving. Whoever sends on it owns the
        # output for teleop_hold seconds, so /cmd_vel never carries two
        # interleaved 20 Hz streams (that is what makes the robot stutter).
        self.teleop_topic = teleop_topic
        self.teleop_hold = teleop_hold
        self.teleop_scale = teleop_scale
        self.last_teleop = 0.0
        if teleop_topic:
            rospy.Subscriber(teleop_topic, Twist, self._cb_teleop, queue_size=1)
            rospy.loginfo('cmd_vel_scale: %s overrides %s for %.2f s',
                          teleop_topic, in_topic, teleop_hold)
        rospy.Timer(rospy.Duration(0.05), self._tick)
        rospy.on_shutdown(self._shutdown)

    def _cb(self, msg):
        if self.teleop_topic and \
                time.time() - self.last_teleop < self.teleop_hold:
            return                      # the keyboard is driving right now
        self.last_rx = time.time()
        self._forward(msg)

    def _cb_teleop(self, msg):
        self.last_teleop = time.time()
        self.last_rx = time.time()      # a hand command also feeds the deadman
        self._forward(msg, teleop=True)

    def _forward(self, msg, teleop=False):
        out = Twist()
        out.linear.x = msg.linear.x * self.linear_scale
        out.linear.y = msg.linear.y * self.linear_scale
        out.linear.z = msg.linear.z * self.linear_scale
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        wz_scale = self.teleop_scale if teleop else self.angular_scale
        if msg.angular.z == 0.0 or self.angular_deadzone <= 0.0:
            out.angular.z = msg.angular.z * wz_scale
        else:
            # inverse of the measured dead zone + gain: the chassis ignores
            # anything below `deadzone` and then multiplies the remainder by
            # 1/angular_scale. Adding the offset back makes a small request
            # produce a small actual rotation instead of nothing.
            out.angular.z = math.copysign(
                self.angular_deadzone + abs(msg.angular.z) * self.angular_scale,
                msg.angular.z)
        # keep inside what the driver/firmware will accept anyway
        out.linear.x = max(-0.45, min(0.45, out.linear.x))
        out.angular.z = max(-2.0, min(2.0, out.angular.z))
        self.last_nonzero = (abs(out.linear.x) > 1e-4 or
                             abs(out.linear.y) > 1e-4 or
                             abs(out.angular.z) > 1e-4)
        if self.last_nonzero:
            self.fired = False
        if not self.dry_run:
            self.pub.publish(out)

    def _tick(self, _event):
        if self.last_nonzero and not self.fired and \
                time.time() - self.last_rx > self.timeout:
            self.fired = True
            rospy.logwarn('cmd_vel_scale: input silent %.2f s, sending zero',
                          time.time() - self.last_rx)
            if not self.dry_run:
                for _ in range(10):
                    self.pub.publish(ZERO)
                    rospy.sleep(0.02)

    def _shutdown(self):
        try:
            for _ in range(10):
                self.pub.publish(ZERO)
                time.sleep(0.02)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in-topic', default='/cmd_vel')
    ap.add_argument('--out-topic', default='/cmd_vel_driver')
    ap.add_argument('--linear-scale', type=float, default=1.0,
                    help='leave at 1.0 unless you have measured otherwise')
    ap.add_argument('--angular-scale', type=float, default=0.709,
                    help='1/1.41 from the 2026-11-03 measurement (0.709)')
    ap.add_argument('--angular-deadzone', type=float, default=0.0,
                    help='rad/s added in the direction of travel to get past '
                         'the chassis dead zone (0.0). Measured: commanding '
                         '0.10 rad/s produces 0.022 actual, i.e. nothing. DWA '
                         'asks for corrections of 0.04-0.08, which land in '
                         'that hole, so the robot drives uncorrected and then '
                         'jumps 9 deg when the error finally grows enough. '
                         'Set 0.15 together with --angular-scale 0.714 to '
                         'invert the measured model: out = 0.15 + w/1.4.')
    ap.add_argument('--timeout', type=float, default=0.5)
    ap.add_argument('--teleop-topic', default='',
                    help='second input for hand driving (e.g. /cmd_vel_teleop); '
                         'empty disables the override')
    ap.add_argument('--teleop-hold', type=float, default=0.3,
                    help='how long a teleop message keeps the planner muted')
    ap.add_argument('--teleop-scale', type=float, default=1.0,
                    help='angular scale for teleop; 1.0 keeps hand turns as '
                         'strong as they are today')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    rospy.init_node('cmd_vel_scale', anonymous=True)
    node = Scaler(args.in_topic, args.out_topic, args.linear_scale,
                  args.angular_scale, args.timeout, args.dry_run,
                  teleop_topic=(args.teleop_topic or None),
                  teleop_hold=args.teleop_hold,
                  teleop_scale=args.teleop_scale)
    node.angular_deadzone = args.angular_deadzone
    rospy.loginfo('cmd_vel_scale %s -> %s  linear x%.3f angular x%.3f '
                  'deadzone %.3f', args.in_topic, args.out_topic,
                  args.linear_scale, args.angular_scale,
                  args.angular_deadzone)
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())

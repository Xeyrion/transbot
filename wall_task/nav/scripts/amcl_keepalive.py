#!/usr/bin/env python3
"""Keep AMCL publishing a fresh map->odom transform while the robot stands still.

AMCL only runs a filter update when the robot has moved update_min_d (0.10 m)
or update_min_a (0.10 rad). The transform it broadcasts carries the stamp of
the last update plus transform_tolerance, so when the robot stands still that
stamp stops advancing. move_base's costmaps then log

    Costmap2DROS transform timeout. Current time: 1793740007.7186,
    global_pose stamp: 1793740007.0142, tolerance: 0.5000

and refuse to plan. The robot looks broken while it is doing exactly what it
was told: nothing.

Calling /request_nomotion_update makes AMCL process the latest scan without
waiting for motion, which advances the stamp. One call per second is enough:
the costmap tolerance is 0.5 s and the transform is stamped
transform_tolerance into the future.

Usage:
    amcl_keepalive.py                 one call per second
    amcl_keepalive.py --rate 2.0
"""

import argparse
import sys

import rospy
from std_srvs.srv import Empty


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=1.0,
                    help='calls per second (1.0). The costmap tolerance is '
                         '0.5 s, so below 2 Hz there is no benefit')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    rospy.init_node('amcl_keepalive', anonymous=True)
    try:
        rospy.wait_for_service('/request_nomotion_update', timeout=10.0)
    except rospy.ROSException:
        print('amcl_keepalive: /request_nomotion_update not available, '
              'is amcl running?')
        return 2

    update = rospy.ServiceProxy('/request_nomotion_update', Empty)
    period = 1.0 / args.rate
    n = 0
    if not args.quiet:
        print('amcl_keepalive: %.1f Hz' % args.rate)
    while not rospy.is_shutdown():
        try:
            update()
            n += 1
        except rospy.ServiceException as exc:
            if not args.quiet:
                print('amcl_keepalive: call failed (%s), retrying' % exc)
            rospy.sleep(1.0)
            continue
        if not args.quiet and n % 30 == 0:
            print('amcl_keepalive: %d updates sent' % n)
        rospy.sleep(period)
    return 0


if __name__ == '__main__':
    sys.exit(main())

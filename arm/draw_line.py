#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""draw_line.py -- three-step straight-line tool.

Step 1 (range):  draw_line.py DIST
    DIST = horizontal j7-axis -> line distance [cm] (you input this).
    Prints the drawable world-height range(s) on that vertical line.

Step 2+3 (draw): draw_line.py DIST Z0 Z1 [--step 0.5]
    Z0/Z1 = heights rel the j7 axis [cm, + = above, - = below].
    Must lie inside a printed range; the pen tip ramps through the
    segment in simulation.

Angle convention: e1=j7-95, rel=j8-180, e2=e1+rel (corrected 2026-10-28).
The archived arm_model.py used rel=j7+j8-275 (wrong: it took the second
link's world angle e2 as the relative angle rel, so j7 was counted twice
and the pen went below ground, e.g. (169.6,30.8) appeared as (26.2,8.0)
when the real pose is (8.57,-15.15)).  This tool uses line_tool.py only.
"""
from __future__ import division, print_function

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import line_tool as L  # noqa: E402

STEP = 0.05          # rad per command
PERIOD = 0.05        # s between commands


def main(argv=None):
    ap = argparse.ArgumentParser(description='draw a vertical line in sim')
    ap.add_argument('dist', type=float, help='j7 axis -> line, cm')
    ap.add_argument('z0', type=float, nargs='?', default=None,
                        help='start height rel j7, cm (negative needs -- : e.g. 26.2 -- -3 7)')
    ap.add_argument('z1', type=float, nargs='?', default=None,
                        help='end height rel j7, cm')
    ap.add_argument('--step', type=float, default=0.5)
    ap.add_argument('--wait', type=float, default=8.0)
    args = ap.parse_args(argv)

    print(L.describe(args.dist))
    if args.z0 is None or args.z1 is None:
        return 0

    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    pts = L.draw_points(args.dist, args.z0, args.z1, step_cm=args.step)
    print('waypoints: %d  from (%.1f, %.1f) to (%.1f, %.1f)'
          % (len(pts), pts[0][0], pts[0][1], pts[-1][0], pts[-1][1]))

    rospy.init_node('draw_line', anonymous=True)
    pub = rospy.Publisher('/arm_controller/command', Float64MultiArray,
                          queue_size=1)
    deadline = time.time() + args.wait
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if time.time() > deadline:
            print('no /arm_controller/command subscriber; is sim running?')
            return 1
        time.sleep(0.1)

    def to_q(j7, j8):
        return (math.radians(j7 - L.J7_ZERO), math.radians(j8 - L.J8_ZERO))

    q1, q2 = 0.0, 0.0
    try:
        js = rospy.wait_for_message('/joint_states', JointState, timeout=3.0)
        idx = dict((n, i) for i, n in enumerate(js.name))
        q1 = js.position[idx['arm_joint1']]
        q2 = js.position[idx['arm_joint2']]
    except Exception as exc:
        print('could not read /joint_states (%s); starting at 0' % exc)

    for (j7, j8) in pts:
        t1, t2 = to_q(j7, j8)
        n = max(1, int(math.ceil(max(abs(t1 - q1), abs(t2 - q2)) / STEP)))
        for k in range(1, n + 1):
            pub.publish(Float64MultiArray(
                data=[q1 + (t1 - q1) * k / n, q2 + (t2 - q2) * k / n]))
            time.sleep(PERIOD)
        q1, q2 = t1, t2
    print('done: drew z %.2f -> %.2f at dist %.2f' % (args.z0, args.z1,
                                                     args.dist))
    time.sleep(0.3)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

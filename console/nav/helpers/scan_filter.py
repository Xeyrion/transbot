#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Remove self-returns from a LaserScan: anything inside the chassis footprint.

Why
---
On 2026-11-02 the scan started reporting 5 beams at 0.111 m around bearing
+110 deg. Converting to base_link put those points at (-0.038, +0.105), i.e.
inside the 0.29 x 0.24 m footprint -- the robot's own antenna / camera bracket
/ cabling, not an obstacle. Two consequences:

  * every in-place turn aborted on the 0.25 m rotation clearance gate
  * gmapping would have baked a phantom obstacle into the map, rotating with
    the robot

The fix is the standard one: a self-filter. Transform each return into
base_link and drop it if it falls inside the footprint polygon. This is what
robot_self_filter does with URDF meshes; a rectangle is enough here because the
chassis is a box and the LiDAR sits at a fixed 0.13 m.

Comparison with the alternatives
--------------------------------
* range gate only (min_range ~0.18 m): simplest, but also throws away real
  obstacles that are genuinely that close, and the robot can approach a wall
  closer than that.
* angular mask: brittle -- it encodes one mount position at one height, and
  breaks the moment the antenna is moved.
* footprint test (this node): drops exactly the robot, keeps everything else.

Usage
    python3 scan_filter.py                      # /scan -> /scan_filtered
    python3 scan_filter.py --report             # also print what is dropped

Then point the consumer at it, e.g. in gmapping.launch:

    <remap from="scan" to="/scan_filtered"/>

Verify with --report that the dropped bearings are the ones you expect. If it
starts dropping 30 beams across a wide arc, the footprint is too big or
something is genuinely attached outside the chassis.
"""
import argparse
import math
import sys

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

# base_link -> laser, matches bringup.launch: -0.03 0 0.13 3.1416 0 0
LASER_X, LASER_Y, LASER_YAW = -0.03, 0.0, math.pi


class Filter(object):
    def __init__(self, in_topic, out_topic, half_x, half_y, inflate,
                 min_range, report):
        self.half_x = half_x + inflate
        self.half_y = half_y + inflate
        self.min_range = min_range
        self.report = report
        self.n_in = 0
        self.dropped = []
        self.pub = rospy.Publisher(out_topic, LaserScan, queue_size=1)
        rospy.Subscriber(in_topic, LaserScan, self._cb, queue_size=1)
        if report:
            rospy.Timer(rospy.Duration(5.0), self._summary)

    def _cb(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        cl, sl = math.cos(LASER_YAW), math.sin(LASER_YAW)
        # errstate: the beam pattern contains inf, and inf * 0 is nan
        with np.errstate(invalid='ignore'):
            xb = r * np.cos(a) * cl - r * np.sin(a) * sl + LASER_X
            yb = r * np.cos(a) * sl + r * np.sin(a) * cl + LASER_Y

        finite = np.isfinite(r)
        inside = (finite &
                  (np.abs(xb) <= self.half_x) & (np.abs(yb) <= self.half_y))
        too_close = finite & (r < self.min_range)
        drop = inside | too_close

        if not drop.any():
            self.pub.publish(msg)
            return

        out = np.array(r, dtype=np.float64)
        out[drop] = float('inf')
        msg.ranges = out.tolist()
        self.pub.publish(msg)

        if self.report:
            self.n_in += 1
            bearing = np.degrees(np.arctan2(-np.sin(a[drop]), -np.cos(a[drop])))
            self.dropped.append(bearing)

    def _summary(self, _event):
        if not self.dropped:
            return
        allb = np.concatenate(self.dropped)
        rospy.loginfo('scan_filter: %d scans, %.1f beams/scan dropped, '
                      'bearings %.0f..%.0f deg (robot frame)',
                      self.n_in, float(allb.size) / max(self.n_in, 1),
                      allb.min(), allb.max())
        self.dropped = []
        self.n_in = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in-topic', default='/scan')
    ap.add_argument('--out-topic', default='/scan_filtered')
    ap.add_argument('--half-x', type=float, default=0.145,
                    help='half length of the chassis (0.145)')
    ap.add_argument('--half-y', type=float, default=0.12,
                    help='half width of the chassis (0.12)')
    ap.add_argument('--inflate', type=float, default=0.015,
                    help='grow the footprint so edge-clipping beams are caught '
                         '(0.015)')
    ap.add_argument('--min-range', type=float, default=0.06,
                    help='also drop returns closer than this (0.06). Keep it '
                         'small: the footprint test is the precise filter')
    ap.add_argument('--report', action='store_true',
                    help='log dropped beams every 5 s so you can check the '
                         'filter is not eating real obstacles')
    args = ap.parse_args()

    rospy.init_node('scan_filter', anonymous=True)
    Filter(args.in_topic, args.out_topic, args.half_x, args.half_y,
           args.inflate, args.min_range, args.report)
    rospy.loginfo('scan_filter %s -> %s, footprint %.3f x %.3f + %.3f m',
                  args.in_topic, args.out_topic,
                  2 * args.half_x, 2 * args.half_y, args.inflate)
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())

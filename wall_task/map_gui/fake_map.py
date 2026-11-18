#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fake_map.py -- synthetic /map + /scan + TF for testing map_gui.py.

No robot needed: builds a 20x12 m room, lets the "known" area grow around a
robot that drives a slow circle, and ray-casts a 360-beam scan against the
walls.  Used to check the rendering and the stats before touching hardware.
"""
import math
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

RES = 0.05
W, H = 400, 240          # 20 x 12 m
OX, OY = -10.0, -6.0
ROOM = (-6.0, -4.0, 6.0, 4.0)      # x0, y0, x1, y1
BOXES = [(-3.0, 1.0, -1.6, 2.2), (2.2, -2.4, 3.6, -1.2)]


def walls_grid():
    """0 free, 100 wall, -1 unknown (outside the room)."""
    g = np.full((H, W), -1, dtype=np.int8)
    xs = OX + (np.arange(W) + 0.5) * RES
    ys = OY + (np.arange(H) + 0.5) * RES
    xx, yy = np.meshgrid(xs, ys)
    inside = ((xx > ROOM[0]) & (xx < ROOM[2]) &
              (yy > ROOM[1]) & (yy < ROOM[3]))
    g[inside] = 0
    edge = ((np.abs(xx - ROOM[0]) < 0.06) | (np.abs(xx - ROOM[2]) < 0.06) |
            (np.abs(yy - ROOM[1]) < 0.06) | (np.abs(yy - ROOM[3]) < 0.06))
    g[edge & inside] = 100
    for (x0, y0, x1, y1) in BOXES:
        b = ((xx > x0) & (xx < x1) & (yy > y0) & (yy < y1))
        g[b] = 100
    return g


def ray_box(x, y, dx, dy, box, rmax):
    x0, y0, x1, y1 = box
    tmin, tmax = 0.0, rmax
    for p, d, lo, hi in ((x, dx, x0, x1), (y, dy, y0, y1)):
        if abs(d) < 1e-9:
            if p < lo or p > hi:
                return None
            continue
        t1 = (lo - p) / d
        t2 = (hi - p) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin if tmin > 0.02 else None


def exit_room(x, y, dx, dy):
    """Distance to the room wall along the ray (the robot is inside)."""
    ts = []
    if dx > 1e-9:
        ts.append((ROOM[2] - x) / dx)
    elif dx < -1e-9:
        ts.append((ROOM[0] - x) / dx)
    if dy > 1e-9:
        ts.append((ROOM[3] - y) / dy)
    elif dy < -1e-9:
        ts.append((ROOM[1] - y) / dy)
    ts = [t for t in ts if t > 0.02]
    return min(ts) if ts else float('inf')


def cast(x, y, a, rmax=8.0):
    dx, dy = math.cos(a), math.sin(a)
    best = exit_room(x, y, dx, dy)
    for b in BOXES:
        h = ray_box(x, y, dx, dy, b, rmax)
        if h is not None:
            best = min(best, h)
    return best if best < rmax else float('inf')


def main():
    rospy.init_node('fake_map')
    pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    pub_scan = rospy.Publisher('/scan', LaserScan, queue_size=5)
    pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=20)

    base = walls_grid()
    t0 = time.time()
    rate = rospy.Rate(10)
    last_map = 0.0
    while not rospy.is_shutdown():
        t = time.time() - t0
        # robot drives a circle of radius 2 m, 0.15 rad/s
        ang = 0.15 * t
        rx, ry = 2.0 * math.cos(ang), 2.0 * math.sin(ang)
        yaw = ang + math.pi / 2.0

        now = rospy.Time.now()
        tfm = TFMessage()
        for parent, child, px, py, pyaw in (
                ('map', 'base_link', rx, ry, yaw),
                ('base_link', 'laser', 0.03, 0.0, 0.0)):
            tr = TransformStamped()
            tr.header.stamp = now
            tr.header.frame_id = parent
            tr.child_frame_id = child
            tr.transform.translation.x = px
            tr.transform.translation.y = py
            tr.transform.rotation.z = math.sin(pyaw / 2.0)
            tr.transform.rotation.w = math.cos(pyaw / 2.0)
            tfm.transforms.append(tr)
        pub_tf.publish(tfm)

        if time.time() - last_map > 1.0:
            last_map = time.time()
            r_known = min(8.0, 1.5 + 0.25 * t)
            g = base.copy()
            xs = OX + (np.arange(W) + 0.5) * RES
            ys = OY + (np.arange(H) + 0.5) * RES
            xx, yy = np.meshgrid(xs, ys)
            visible = (xx - rx) ** 2 + (yy - ry) ** 2 < r_known ** 2
            g[~visible] = -1
            msg = OccupancyGrid()
            msg.header.stamp = now
            msg.header.frame_id = 'map'
            msg.info.resolution = RES
            msg.info.width = W
            msg.info.height = H
            msg.info.origin.position.x = OX
            msg.info.origin.position.y = OY
            msg.info.origin.orientation.w = 1.0
            msg.data = g.reshape(-1).tolist()
            pub_map.publish(msg)

        s = LaserScan()
        s.header.stamp = now
        s.header.frame_id = 'laser'
        s.angle_min = -math.pi
        s.angle_max = math.pi
        s.angle_increment = 2 * math.pi / 360
        s.range_min = 0.15
        s.range_max = 8.0
        s.ranges = [cast(rx, ry, -math.pi + i * s.angle_increment + yaw)
                    for i in range(360)]
        pub_scan.publish(s)
        rate.sleep()


if __name__ == '__main__':
    main()

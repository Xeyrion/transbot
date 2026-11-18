#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_gui.py -- live gmapping view without rviz.

Why it can look close to rviz: the whole frame (map + laser + trail + robot)
is composited into one PIL image at 2x and downscaled, so walls and dots get
antialiased.  Drawing 400 canvas ovals, as the first version did, gives hard
jagged dots and a flat picture.

What it cannot do: rviz is OpenGL.  No 3D view, no point-cloud rendering, no
transparency layers.  For a 2D map those do not matter; the top-down view is
the useful one anyway.

Mouse: wheel = zoom at cursor, drag = pan, double click = reset.
"""

from __future__ import print_function

import argparse
import json
import math
import os
import re
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
from PIL import Image, ImageDraw, ImageTk

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
from tf2_msgs.msg import TFMessage

SS = 2                      # supersampling factor for the composite
MAP_W, MAP_H = 860, 660
PANEL_W = 360

PALETTES = {
    'light': {'unknown': (239, 238, 232), 'free': (255, 255, 255),
              'occ': (38, 38, 36), 'mid': (150, 148, 140),
              'bg': '#faf9f6', 'scan': (226, 75, 74), 'trail': (55, 138, 221),
              'robot': (239, 159, 39), 'robot_edge': (133, 79, 11)},
    # rviz's own map display: free light grey, occupied black, unknown a
    # middle grey.  Unknown must be lighter than the walls or the outline of
    # the room disappears into the background.
    'dark': {'unknown': (92, 92, 90), 'free': (203, 203, 200),
             'occ': (5, 5, 5), 'mid': (140, 140, 138),
             'bg': '#2b2b2a', 'scan': (255, 92, 92), 'trail': (90, 170, 255),
             'robot': (255, 176, 60), 'robot_edge': (90, 50, 10)},
}


STATUS_NAMES = {0: 'pending', 1: 'active', 2: 'preempted', 3: 'succeeded',
                4: 'aborted', 5: 'rejected', 6: 'preempting', 7: 'recalling',
                8: 'recalled', 9: 'lost'}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TFLite(object):
    """Just enough TF: compose map -> base_link -> laser from the latest edges.

    tf2_ros is not usable here: tf2_py ships a Python 2 extension and this
    GUI runs under Python 3.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.edges = {}

    @staticmethod
    def _norm(frame):
        # the robot publishes some frames as "/laser" while the scan header
        # says "laser"; tf2 strips the leading slash, this lite version has to
        return frame.lstrip('/') if frame else frame

    def cb(self, msg):
        with self.lock:
            for tr in msg.transforms:
                t = tr.transform.translation
                q = tr.transform.rotation
                self.edges[self._norm(tr.child_frame_id)] = (
                    self._norm(tr.header.frame_id), t.x, t.y, yaw_of(q))

    @staticmethod
    def _compose(a, b):
        x1, y1, a1 = a
        x2, y2, a2 = b
        return (x1 + math.cos(a1) * x2 - math.sin(a1) * y2,
                y1 + math.sin(a1) * x2 + math.cos(a1) * y2,
                a1 + a2)

    def lookup(self, target, source):
        acc = (0.0, 0.0, 0.0)
        target = self._norm(target)
        cur = self._norm(source)
        with self.lock:
            for _ in range(12):
                if cur == target:
                    return acc
                e = self.edges.get(cur)
                if e is None:
                    return None
                parent, x, y, a = e
                # T_target_child = T_target_parent * T_parent_child, so the
                # new (higher) edge goes on the LEFT.  With acc on the left
                # the chain works only while every yaw is zero -- the laser
                # frame has a 180 deg yaw, and that mirrored the scan through
                # the origin (points appeared on the far side of the robot).
                acc = self._compose((x, y, a), acc)
                cur = parent
        return None


class LiveMap(object):
    """OccupancyGrid -> PIL image, plus the view transform (zoom / pan)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.msg = None
        self.img = None
        self.res = 0.05
        self.ox = self.oy = 0.0
        self.w = self.h = 0
        self.dirty = True
        self.need_fit = True
        self.zoom = 1.0
        self.panx = self.pany = 0.0
        self.s = 1.0
        self.offx = self.offy = 0.0
        self.photo = None
        self.palette = PALETTES['light']
        self.cw = self.ch = 0
        self.c0 = self.c1 = self.r0 = self.r1 = 0

    def set_palette(self, name):
        with self.lock:
            self.palette = PALETTES[name]
            if self.msg is not None:
                self._rebuild_locked()
            self.dirty = True

    def _bbox_locked(self):
        arr = np.array(self.msg.data, dtype=np.int16).reshape(self.h, self.w)
        known = arr != -1
        if not known.any():
            self.c0, self.c1, self.r0, self.r1 = 0, self.w - 1, 0, self.h - 1
            return
        rows = np.any(known, axis=1)
        cols = np.any(known, axis=0)
        self.r0 = int(np.argmax(rows))
        self.r1 = int(len(rows) - 1 - np.argmax(rows[::-1]))
        self.c0 = int(np.argmax(cols))
        self.c1 = int(len(cols) - 1 - np.argmax(cols[::-1]))

    def _rebuild_locked(self):
        arr = np.array(self.msg.data, dtype=np.int16).reshape(self.h, self.w)
        pal = self.palette
        rgb = np.empty((self.h, self.w, 3), dtype=np.uint8)
        rgb[:, :] = pal['unknown']
        rgb[arr == 0] = pal['free']
        rgb[arr >= 50] = pal['occ']
        mid = (arr > 0) & (arr < 50)
        rgb[mid] = pal['mid']
        self.img = Image.fromarray(rgb[::-1])

    def update(self, msg):
        w, h = msg.info.width, msg.info.height
        if w <= 0 or h <= 0:
            return
        with self.lock:
            self.msg = msg
            self.res = msg.info.resolution
            self.ox = msg.info.origin.position.x
            self.oy = msg.info.origin.position.y
            self.w, self.h = w, h
            self._rebuild_locked()
            self._bbox_locked()
            self.dirty = True
            self.need_fit = True

    # -- view ----------------------------------------------------------
    def _fit(self, cw, ch, margin=16):
        # frame the KNOWN area, not the whole grid: gmapping grows the grid
        # far beyond what has been seen, and fitting that leaves the map as a
        # small island in a sea of grey
        bw = max(self.c1 - self.c0 + 1, 1)
        bh = max(self.r1 - self.r0 + 1, 1)
        s0 = max(0.05, min((cw - 2 * margin) / float(bw),
                           (ch - 2 * margin) / float(bh)))
        self.s = s0 * self.zoom
        self.offx = (cw - bw * self.s) / 2.0 + self.panx - self.c0 * self.s
        self.offy = (ch - bh * self.s) / 2.0 + self.pany - self.r0 * self.s
        self.cw, self.ch = cw, ch
        self.need_fit = False

    def world_to_canvas(self, x, y):
        col = (x - self.ox) / self.res
        row = self.h - 1 - (y - self.oy) / self.res
        return (self.offx + col * self.s, self.offy + row * self.s)

    def canvas_to_world(self, px, py):
        col = (px - self.offx) / self.s
        row = (py - self.offy) / self.s
        return (self.ox + col * self.res,
                self.oy + (self.h - 1 - row) * self.res)

    def zoom_at(self, px, py, factor):
        wx, wy = self.canvas_to_world(px, py)
        self.zoom = max(0.5, min(12.0, self.zoom * factor))
        self._fit(self.cw, self.ch)
        nx, ny = self.world_to_canvas(wx, wy)
        self.panx += px - nx
        self.pany += py - ny
        self._fit(self.cw, self.ch)
        self.dirty = True

    def pan(self, dx, dy):
        self.panx += dx
        self.pany += dy
        self._fit(self.cw, self.ch)
        self.dirty = True

    def reset_view(self):
        self.zoom = 1.0
        self.panx = self.pany = 0.0
        self._fit(self.cw, self.ch)
        self.dirty = True

    # -- composite -----------------------------------------------------
    def render(self, cw, ch, scan=None, pose=None, trail=None,
               plan=None, goal=None):
        if cw < 20 or ch < 20:
            return None
        with self.lock:
            if self.img is None:
                return None
            if self.need_fit or cw != self.cw or ch != self.ch:
                self._fit(cw, ch)
            if not self.dirty and self.photo is not None:
                return self.photo

            s = self.s * SS
            offx = self.offx * SS
            offy = self.offy * SS
            W = cw * SS
            H = ch * SS
            canvas = Image.new('RGB', (W, H), self.palette['unknown'])
            col0 = int(math.floor(-offx / s))
            col1 = int(math.ceil((W - offx) / s))
            row0 = int(math.floor(-offy / s))
            row1 = int(math.ceil((H - offy) / s))
            c0, c1 = max(0, col0), min(self.w, col1)
            r0, r1 = max(0, row0), min(self.h, row1)
            if c1 > c0 and r1 > r0:
                crop = self.img.crop((c0, r0, c1, r1))
                crop = crop.resize((max(1, int((c1 - c0) * s)),
                                    max(1, int((r1 - r0) * s))),
                                   Image.BILINEAR)
                canvas.paste(crop, (int(offx + c0 * s), int(offy + r0 * s)))
            draw = ImageDraw.Draw(canvas)
            pal = self.palette

            if trail and len(trail) > 1:
                step = max(1, len(trail) // 1200)
                pts = [self._w2c(x, y, offx, offy, s)
                       for x, y in trail[::step]]
                draw.line(pts, fill=pal['trail'], width=max(1, SS // 2),
                          joint='curve')

            if plan and len(plan) > 1:
                pts = [self._w2c(x, y, offx, offy, s) for x, y in plan]
                draw.line(pts, fill=(70, 180, 90), width=max(1, SS),
                          joint='curve')

            if goal is not None:
                gx, gy = self._w2c(goal[0], goal[1], offx, offy, s)
                rr = 5 * SS
                draw.ellipse((gx - rr, gy - rr, gx + rr, gy + rr),
                             outline=(70, 170, 230), width=SS)

            if scan:
                r = max(1.0, 1.1 * SS)
                for x, y in scan:
                    px, py = self._w2c(x, y, offx, offy, s)
                    if -5 < px < W + 5 and -5 < py < H + 5:
                        draw.ellipse((px - r, py - r, px + r, py + r),
                                     fill=pal['scan'])

            if pose is not None:
                px, py = self._w2c(pose[0], pose[1], offx, offy, s)
                ln = 8 * SS
                ca, sa = math.cos(pose[2]), math.sin(pose[2])
                tri = [(px + ln * ca, py - ln * sa),
                       (px - ln * 0.55 * ca + ln * 0.42 * sa,
                        py + ln * 0.55 * sa + ln * 0.42 * ca),
                       (px - ln * 0.55 * ca - ln * 0.42 * sa,
                        py + ln * 0.55 * sa - ln * 0.42 * ca)]
                draw.polygon(tri, fill=pal['robot'], outline=pal['robot_edge'])

            canvas = canvas.resize((cw, ch), Image.BILINEAR)
            self.photo = ImageTk.PhotoImage(canvas)
            self.dirty = False
            return self.photo

    def _w2c(self, x, y, offx, offy, s):
        col = (x - self.ox) / self.res
        row = self.h - 1 - (y - self.oy) / self.res
        return (offx + col * s, offy + row * s)

    # -- queries -------------------------------------------------------
    def clear(self, x, y, radius=0.30):
        """True when every cell within `radius` is known and free.

        Clicking a goal on unknown space would let move_base plan straight
        through the void (allow_unknown is on), so refuse it here instead.
        """
        with self.lock:
            if self.msg is None:
                return False, 'no map'
            data = np.array(self.msg.data, dtype=np.int16).reshape(self.h, self.w)
            res = self.res
        col = int((x - self.ox) / res)
        row = int((y - self.oy) / res)
        k = max(1, int(round(radius / res)))
        r0, r1 = max(0, row - k), min(self.h, row + k + 1)
        c0, c1 = max(0, col - k), min(self.w, col + k + 1)
        if r1 <= r0 or c1 <= c0:
            return False, 'outside the map'
        sub = data[r0:r1, c0:c1]
        if (sub == 100).any():
            return False, 'obstacle'
        if (sub == -1).any():
            return False, 'unknown'
        return True, ''

    # -- stats ---------------------------------------------------------
    def stats(self):
        with self.lock:
            if self.msg is None:
                return None
            arr = np.array(self.msg.data, dtype=np.int16).reshape(self.h, self.w)
            res = self.res
        known = arr != -1
        n = int(known.sum())
        if not n:
            return None
        rows = np.any(known, axis=1)
        cols = np.any(known, axis=0)
        r0, r1 = int(np.argmax(rows)), int(len(rows) - 1 - np.argmax(rows[::-1]))
        c0, c1 = int(np.argmax(cols)), int(len(cols) - 1 - np.argmax(cols[::-1]))
        bbox = (c1 - c0 + 1) * (r1 - r0 + 1)
        return {'area': n * res * res,
                'cover': 100.0 * n / max(bbox, 1),
                'free': 100.0 * float((arr == 0).sum()) / n,
                'w': (c1 - c0 + 1) * res,
                'h': (r1 - r0 + 1) * res,
                'cells': (self.w, self.h)}


class App(object):
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.map = LiveMap()
        self.lock = threading.Lock()
        self.scan = None
        self.scan_times = []
        self.pose = None
        self.trail = []
        self.trail_len = 0.0
        self.paused = False
        self.teleop_on = False
        self.keys = set()
        self._drag = None
        self._press = None
        self.nav_on = bool(getattr(args, 'nav', False))
        self.plan = None
        self.goal = None
        self.mb_status = 'idle'

        root.title('建图 · Transbot')
        root.geometry('1240x800')
        root.minsize(1120, 720)
        self._build()

        rospy.init_node('map_gui', anonymous=True, disable_signals=True)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped,
                                        queue_size=1)
        self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID,
                                          queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self.cb_map, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.cb_scan, queue_size=3)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.cb_status,
                         queue_size=5)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.cb_plan,
                         queue_size=2)
        self.tf = TFLite()
        rospy.Subscriber('/tf', TFMessage, self.tf.cb, queue_size=50)
        rospy.Subscriber('/tf_static', TFMessage, self.tf.cb, queue_size=50)
        threading.Thread(target=rospy.spin, daemon=True).start()

        if self.args.dark:
            self.toggle_theme()
        root.bind('<KeyPress>', self.on_key)
        root.bind('<KeyRelease>', self.on_key_up)
        root.bind('<FocusOut>', lambda e: self.stop())
        self.tick()

    # -- layout --------------------------------------------------------
    def _build(self):
        body = tk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(body, width=MAP_W, height=MAP_H,
                                bg=PALETTES['light']['bg'],
                                highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky='nsew')
        self.canvas.bind('<ButtonPress-1>', self.on_press)
        self.canvas.bind('<B1-Motion>', self.on_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_release)
        self.canvas.bind('<Double-Button-1>', lambda e: self.map.reset_view())
        self.canvas.bind('<Button-3>', self.on_right)
        self.canvas.bind('<Button-4>', lambda e: self.map.zoom_at(e.x, e.y, 1.2))
        self.canvas.bind('<Button-5>', lambda e: self.map.zoom_at(e.x, e.y, 1 / 1.2))
        self.canvas.bind('<MouseWheel>',
                         lambda e: self.map.zoom_at(e.x, e.y,
                                                    1.2 if e.delta > 0 else 1 / 1.2))

        right = tk.Frame(body, width=PANEL_W)
        right.grid(row=0, column=1, sticky='ns', padx=(10, 0))
        right.grid_propagate(False)

        top = tk.Frame(right)
        top.pack(fill=tk.X)
        self.btn_pause = tk.Button(top, text='暂停', width=6,
                                   command=self.toggle_pause)
        self.btn_pause.pack(side=tk.LEFT)
        tk.Button(top, text='重置视图', width=8,
                  command=self.map.reset_view).pack(side=tk.LEFT, padx=6)
        tk.Button(top, text='保存', width=6,
                  command=self.save_map).pack(side=tk.LEFT)

        row = tk.Frame(right)
        row.pack(fill=tk.X, pady=(6, 0))
        tk.Label(row, text='地图名').pack(side=tk.LEFT)
        self.name_var = tk.StringVar(value=time.strftime('lab_%m%d'))
        tk.Entry(row, textvariable=self.name_var).pack(
            side=tk.LEFT, padx=6, fill=tk.X, expand=True)

        self.state_lbl = tk.Label(right, text='等待 /map …', anchor=tk.W,
                                  fg='#5f5e5a')
        self.state_lbl.pack(fill=tk.X, pady=(8, 4))

        self.vals = {}
        rows = [('area', '已知面积'), ('cover', '覆盖率'), ('free', '空闲占比'),
                ('size', '地图尺寸'), ('rate', '扫描频率'), ('pts', '每帧点数'),
                ('pose', '当前位置'), ('dist', '里程'), ('zoom', '缩放')]
        # the nav row only exists in slam_nav's build (--nav)
        if self.nav_on:
            rows.append(('nav', '导航'))
        box = tk.Frame(right)
        box.pack(fill=tk.X, pady=(6, 0))
        for i, (key, label) in enumerate(rows):
            tk.Label(box, text=label, fg='#888780', anchor=tk.W).grid(
                row=i, column=0, sticky=tk.W, pady=3)
            v = tk.Label(box, text='--', anchor=tk.E, font=('TkFixedFont', 10))
            v.grid(row=i, column=1, sticky=tk.E)
            self.vals[key] = v
        box.columnconfigure(0, weight=1)
        box.columnconfigure(1, weight=1)

        ttk.Separator(right, orient='horizontal').pack(fill=tk.X, pady=10)

        self.dark = tk.BooleanVar(value=bool(self.args.dark))
        tk.Checkbutton(right, text='深色（接近 rviz）', variable=self.dark,
                       command=self.toggle_theme).pack(anchor=tk.W)

        if self.nav_on:
            self.nav_var = tk.BooleanVar(value=True)
            tk.Checkbutton(right, text='点地图发目标（边建图边导航）',
                           variable=self.nav_var,
                           command=self.toggle_nav).pack(anchor=tk.W)

        self.teleop_var = tk.BooleanVar(value=False)
        tk.Checkbutton(right, text='启用遥控（WASD / 方向键）',
                       variable=self.teleop_var,
                       command=self.toggle_teleop).pack(anchor=tk.W)
        sp = tk.Frame(right)
        sp.pack(fill=tk.X, pady=(6, 0))
        tk.Label(sp, text='速度 m/s').pack(side=tk.LEFT)
        self.speed = tk.DoubleVar(value=0.12)
        tk.Spinbox(sp, from_=0.04, to=0.30, increment=0.02, width=6,
                   textvariable=self.speed).pack(side=tk.LEFT, padx=6)

        tk.Label(right, text='保存目录（相对项目）', fg='#888780',
                 anchor=tk.W).pack(fill=tk.X, pady=(12, 2))
        tk.Label(right, text=self.args.maps_rel, anchor=tk.W, fg='#5f5e5a',
                 wraplength=PANEL_W - 10, justify=tk.LEFT).pack(fill=tk.X)
        tk.Label(right, text=self.args.maps_dir, anchor=tk.W, fg='#b4b2a9',
                 wraplength=PANEL_W - 10, justify=tk.LEFT,
                 font=('TkFixedFont', 8)).pack(fill=tk.X)

        self.status = tk.Label(self.root, text='', anchor=tk.W, fg='#5f5e5a')
        self.status.pack(fill=tk.X, padx=12, pady=(0, 6))

    # -- callbacks -----------------------------------------------------
    def cb_map(self, msg):
        if not self.paused:
            self.map.update(msg)

    def cb_scan(self, msg):
        tr = self.tf.lookup('map', msg.header.frame_id)
        if tr is None:
            return
        tx, ty, yaw = tr
        ca, sa = math.cos(yaw), math.sin(yaw)
        pts = []
        a = msg.angle_min
        inc = msg.angle_increment
        rmax = msg.range_max
        for i, r in enumerate(msg.ranges):
            if r is None or r != r or r <= 0.02 or r >= rmax:
                continue
            lx = r * math.cos(a + i * inc)
            ly = r * math.sin(a + i * inc)
            pts.append((tx + ca * lx - sa * ly, ty + sa * lx + ca * ly))
        with self.lock:
            self.scan = pts
            self.scan_times.append(time.time())
            del self.scan_times[:-40]
            self.map.dirty = True

    def cb_status(self, msg):
        if not msg.status_list:
            return
        self.mb_status = STATUS_NAMES.get(msg.status_list[-1].status, '?')

    def cb_plan(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    # -- mouse ---------------------------------------------------------
    def on_press(self, ev):
        self._drag = (ev.x, ev.y)
        self._press = (ev.x, ev.y)

    def on_drag(self, ev):
        if self._drag is None:
            return
        dx = ev.x - self._drag[0]
        dy = ev.y - self._drag[1]
        self._drag = (ev.x, ev.y)
        self.map.pan(dx, dy)

    def on_release(self, ev):
        self._drag = None
        press, self._press = self._press, None
        if press is None or not self.nav_on:
            return
        # a click that moved is a pan, not a goal
        if math.hypot(ev.x - press[0], ev.y - press[1]) > 6:
            return
        wx, wy = self.map.canvas_to_world(ev.x, ev.y)
        ok, why = self.map.clear(wx, wy, 0.30)
        if not ok:
            self.status.config(text='不能去那里：%s' % why)
            return
        # ...and the other way round: sending a goal turns teleop off.
        if self.teleop_on:
            self.teleop_var.set(False)
            self.teleop_on = False
            self.stop()
        yaw = 0.0
        if self.pose is not None:
            yaw = math.atan2(wy - self.pose[1], wx - self.pose[0])
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.pose.position.x = wx
        m.pose.position.y = wy
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(m)
        self.goal = (wx, wy, yaw)
        self.status.config(text='目标 (%.2f, %.2f) %.0f°'
                                % (wx, wy, math.degrees(yaw)))

    def on_right(self, ev):
        self.pub_cancel.publish(GoalID())
        self.goal = None
        self.plan = None
        self.status.config(text='目标已取消')

    # -- actions -------------------------------------------------------
    def toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.config(text='继续' if self.paused else '暂停')

    def toggle_theme(self):
        name = 'dark' if self.dark.get() else 'light'
        self.map.set_palette(name)
        self.canvas.config(bg=PALETTES[name]['bg'])

    def toggle_teleop(self):
        self.teleop_on = bool(self.teleop_var.get())
        if not self.teleop_on:
            self.stop()
            return
        # Teleop and navigation both write /cmd_vel.  With both on, the driver
        # gets move_base's command and ours interleaved at 10 Hz, which is the
        # stutter you see.  Teleop takes over: drop the goal first.
        if self.goal is not None or self.plan:
            self.pub_cancel.publish(GoalID())
            self.goal = None
            self.plan = None
            self.status.config(text='遥控接管，导航目标已取消')

    def toggle_nav(self):
        self.nav_on = bool(self.nav_var.get())
        if not self.nav_on:
            self.pub_cancel.publish(GoalID())
            self.goal = None
            self.plan = None

    def stop(self):
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())

    def on_key(self, ev):
        k = (ev.keysym or '').lower()
        if k in ('w', 'a', 's', 'd', 'up', 'down', 'left', 'right'):
            self.keys.add(k)

    def on_key_up(self, ev):
        self.keys.discard((ev.keysym or '').lower())

    def _tick_teleop(self):
        if not self.teleop_on:
            return
        v = w = 0.0
        s = float(self.speed.get())
        if 'w' in self.keys or 'up' in self.keys:
            v += s
        if 's' in self.keys or 'down' in self.keys:
            v -= s
        if 'a' in self.keys or 'left' in self.keys:
            w += s * 3.0
        if 'd' in self.keys or 'right' in self.keys:
            w -= s * 3.0
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.cmd_pub.publish(t)

    def save_map(self):
        name = (self.name_var.get() or '').strip()
        if not name:
            self.status.config(text='先给地图取个名字')
            return
        if not re.match(r'^[\w][\w\-. ]*$', name):
            self.status.config(text='名字只能字母、数字、下划线、横线、点、空格')
            return
        out = os.path.join(self.args.maps_dir, name)
        if os.path.exists(out + '.yaml'):
            if not messagebox.askyesno('覆盖', '%s.yaml 已存在，覆盖？' % name):
                return
        try:
            os.makedirs(self.args.maps_dir, exist_ok=True)
            r = subprocess.run(['rosrun', 'map_server', 'map_saver', '-f', out],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               timeout=30)
        except Exception as exc:
            self.status.config(text='保存失败：%s' % exc)
            return
        if r.returncode != 0 or not os.path.exists(out + '.yaml'):
            self.status.config(text='保存失败：%s'
                               % r.stdout.decode('utf-8', 'replace')[-120:])
            return
        # map_saver writes an absolute image path; rewrite it to the bare
        # file name so the whole maps/ folder can be moved elsewhere
        try:
            yml = out + '.yaml'
            with open(yml) as f:
                lines = f.readlines()
            with open(yml, 'w') as f:
                for ln in lines:
                    if ln.startswith('image:'):
                        f.write('image: %s.pgm\n' % name)
                    else:
                        f.write(ln)
        except Exception:
            pass
        st = self.map.stats()
        meta = {'name': name,
                'saved': time.strftime('%Y-%m-%d %H:%M:%S'),
                'resolution': self.map.res,
                'width': self.map.w, 'height': self.map.h,
                'pose': None if self.pose is None else [round(v, 3) for v in self.pose],
                'odom_m': round(self.trail_len, 1)}
        if st:
            meta.update({'area_m2': round(st['area'], 1),
                         'coverage_pct': round(st['cover'], 1),
                         'free_pct': round(st['free'], 1),
                         'size_m': [round(st['w'], 1), round(st['h'], 1)]})
        try:
            with open(out + '.json', 'w') as f:
                json.dump(meta, f, ensure_ascii=False, indent=1)
        except Exception:
            pass
        self.status.config(text='已保存 %s/%s.yaml' % (self.args.maps_rel, name))

    # -- redraw --------------------------------------------------------
    def tick(self):
        self.root.after(100, self.tick)     # 10 Hz is plenty for mapping
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        pose = (self.tf.lookup('map', 'base_link')
                or self.tf.lookup('map', 'base_footprint'))
        if pose is not None:
            self.pose = pose
            if not self.trail or math.hypot(pose[0] - self.trail[-1][0],
                                            pose[1] - self.trail[-1][1]) > 0.05:
                if self.trail:
                    self.trail_len += math.hypot(
                        pose[0] - self.trail[-1][0],
                        pose[1] - self.trail[-1][1])
                self.trail.append((pose[0], pose[1]))
                del self.trail[:-6000]

        with self.lock:
            scan = None if self.scan is None else list(self.scan)
        photo = self.map.render(cw, ch, scan=scan, pose=pose,
                                trail=self.trail, plan=self.plan,
                                goal=self.goal)
        self.canvas.delete('all')
        if photo is not None:
            self.canvas.create_image(0, 0, anchor=tk.NW, image=photo)
            self.canvas.image = photo
        else:
            self.canvas.create_text(cw // 2, ch // 2, text='等待 /map …',
                                    fill='#888780')
        self._redraw_stats(pose)
        self._tick_teleop()

    def _redraw_stats(self, pose):
        st = self.map.stats()
        if st is None:
            self.state_lbl.config(text='等待 /map …')
            return
        self.state_lbl.config(text='gmapping 运行中 · %dx%d 格' % st['cells'])
        self.vals['area'].config(text='%.1f m²' % st['area'])
        self.vals['cover'].config(text='%.1f %%' % st['cover'])
        self.vals['free'].config(text='%.1f %%' % st['free'])
        self.vals['size'].config(text='%.1f × %.1f m' % (st['w'], st['h']))
        with self.lock:
            times = list(self.scan_times)
            n = 0 if self.scan is None else len(self.scan)
        now = time.time()
        recent = [t for t in times if now - t <= 3.0]
        self.vals['rate'].config(text='%.1f Hz' % (len(recent) / 3.0))
        self.vals['pts'].config(text='%d' % n)
        if pose is not None:
            self.vals['pose'].config(text='%+.2f, %+.2f  %.0f°'
                                     % (pose[0], pose[1],
                                        math.degrees(pose[2])))
        self.vals['dist'].config(text='%.1f m' % self.trail_len)
        self.vals['zoom'].config(text='%.1f ×' % self.map.zoom)
        if 'nav' in self.vals:
            self.vals['nav'].config(text=self.mb_status)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--maps-dir', '--map-dir', dest='maps_dir', default='maps',
                    help='where maps are saved; a relative path is resolved '
                         'against the project root, so the whole folder can be '
                         'moved without editing anything')
    ap.add_argument('--nav', action='store_true',
                    help='click on the map to send a move_base goal (needs '
                         'move_base running on the live /map)')
    ap.add_argument('--dark', action='store_true',
                    help='start with the rviz-like dark palette')
    args = ap.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    args.maps_dir = os.path.expanduser(args.maps_dir)
    if not os.path.isabs(args.maps_dir):
        args.maps_dir = os.path.normpath(os.path.join(root, args.maps_dir))
    # what the panel shows: the path as written, relative when it is inside
    # the project
    try:
        rel = os.path.relpath(args.maps_dir, root)
        args.maps_rel = rel if not rel.startswith('..') else args.maps_dir
    except ValueError:
        args.maps_rel = args.maps_dir
    os.makedirs(args.maps_dir, exist_ok=True)
    root = tk.Tk()
    app = App(root, args)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

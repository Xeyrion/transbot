#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Map widget for the console: OccupancyGrid -> PIL image -> Tk canvas.

Split out of the GUI file because it is self-contained: it knows about
occupancy grids, a view transform (zoom / pan / rotate) and a handful of
overlays (laser, trail, plan, goal, robot). It knows nothing about ROS nodes,
modes or buttons.

Three things here exist for a reason and should not be "simplified":

1. The whole frame is composited into one PIL image at 2x and then downscaled.
   Drawing 400 canvas ovals, as the first version did, gives hard jagged dots
   and a flat picture.

2. The view is fitted to the KNOWN area, not to the whole grid. gmapping
   grows the grid far beyond what has been seen, so fitting the grid leaves
   the map as a small island in a sea of grey.

3. The transform is centre-based (bbox centre -> canvas centre) rather than
   offset-based, because that is what makes rotation cheap: rotate the map
   raster once, keep rotating the coordinates with a 2x2 matrix.
"""

import math
import threading

import numpy as np
from PIL import Image, ImageDraw, ImageTk

SS = 2                      # supersampling factor for the composite

PALETTES = {
    # matches the console's own dark chrome (#1e2228)
    'dark': {'unknown': (74, 79, 86), 'free': (188, 193, 200),
             'occ': (12, 13, 15), 'mid': (128, 133, 140),
             'scan': (255, 92, 92), 'trail': (90, 170, 255),
             'plan': (70, 190, 110), 'goal': (70, 170, 230),
             'robot': (255, 176, 60), 'robot_edge': (90, 50, 10)},
    'light': {'unknown': (232, 231, 226), 'free': (255, 255, 255),
              'occ': (38, 38, 36), 'mid': (150, 148, 140),
              'scan': (214, 60, 60), 'trail': (55, 138, 221),
              'plan': (45, 150, 80), 'goal': (40, 120, 200),
              'robot': (222, 140, 20), 'robot_edge': (120, 70, 10)},
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TFLite(object):
    """Just enough TF: walk parent links and compose 2D transforms.

    tf2_ros is not usable here: tf2_py ships a Python 2 extension and this GUI
    runs under Python 3 (ImportError: PyInit__tf2).
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
                # new (higher) edge goes on the LEFT. With acc on the left the
                # chain works only while every yaw is zero -- the laser frame
                # has a 180 deg yaw, and that mirrored the scan through the
                # origin (points appeared on the far side of the robot).
                acc = self._compose((x, y, a), acc)
                cur = parent
        return None


class LiveMap(object):
    """OccupancyGrid -> PIL image, plus the view transform (zoom/pan/rotate)."""

    def __init__(self, palette='dark'):
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
        self.rot = 0.0                     # radians, view rotation
        self.s = 1.0
        self.cw = self.ch = 0
        self.c0 = self.c1 = self.r0 = self.r1 = 0
        self.xc = self.yc = 0.0            # world centre of the known area
        self.photo = None
        self.palette = PALETTES[palette]
        self._rimg = None                  # cached rotated raster
        self._rimg_key = None

    # -- ingest --------------------------------------------------------
    def set_palette(self, name):
        with self.lock:
            self.palette = PALETTES[name]
            if self.msg is not None:
                self._rebuild_locked()
            self.dirty = True

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
            self._rimg = None
            self.dirty = True
            self.need_fit = True

    def _bbox_locked(self):
        arr = np.array(self.msg.data, dtype=np.int16).reshape(self.h, self.w)
        known = arr != -1
        if not known.any():
            self.c0, self.c1, self.r0, self.r1 = 0, self.w - 1, 0, self.h - 1
            return
        rows = np.any(known, axis=1)
        cols = np.any(known, axis=0)
        # numpy rows count from the BOTTOM of the grid (data[0] is the origin
        # corner), but everything downstream -- the PIL image, the view
        # offsets -- counts from the TOP. Convert here, once: with a
        # bottom-up r0 the view is off by (h - r0 - r1) rows, which is
        # invisible on a symmetric gmapping grid and very visible on a fixed
        # map from map_server (the room gets pushed off the bottom edge).
        r_bu0 = int(np.argmax(rows))
        r_bu1 = int(len(rows) - 1 - np.argmax(rows[::-1]))
        self.r0 = self.h - 1 - r_bu1
        self.r1 = self.h - 1 - r_bu0
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
        # row 0 of an OccupancyGrid is the bottom row of the image
        self.img = Image.fromarray(rgb[::-1])

    # -- rotation ------------------------------------------------------
    def set_rotation(self, deg):
        with self.lock:
            self.rot = math.radians(deg % 360.0)
            self._rimg = None
            self.need_fit = True
            self.dirty = True

    def rotation_deg(self):
        return round(math.degrees(self.rot), 1)

    def _raster_locked(self):
        """The known area, rotated. Cached: rotating 800x800 per frame is
        pointless when the angle only changes when a button is pressed."""
        key = (round(math.degrees(self.rot), 2), id(self.msg))
        if self._rimg is not None and self._rimg_key == key:
            return self._rimg
        box = (self.c0, self.r0, self.c1 + 1, self.r1 + 1)
        crop = self.img.crop(box)
        if abs(self.rot) > 1e-6:
            # expand=True 会在四角补黑，必须显式补成"未知"色，否则旋转后
            # 地图外面是一圈纯黑，看着像墙
            crop = crop.rotate(math.degrees(self.rot), resample=Image.BICUBIC,
                               expand=True, fillcolor=self.palette['unknown'])
        self._rimg = crop
        self._rimg_key = key
        return crop

    # -- view ----------------------------------------------------------
    def _fit(self, cw, ch, margin=16):
        bw = (self.c1 - self.c0 + 1) * self.res
        bh = (self.r1 - self.r0 + 1) * self.res
        ca, sa = abs(math.cos(self.rot)), abs(math.sin(self.rot))
        rw = bw * ca + bh * sa
        rh = bw * sa + bh * ca
        s0 = max(0.02, min((cw - 2 * margin) / max(rw, 1e-3),
                           (ch - 2 * margin) / max(rh, 1e-3)))
        self.s = s0 * self.zoom
        self.cw, self.ch = cw, ch
        self.xc = self.ox + (self.c0 + (self.c1 - self.c0 + 1) / 2.0) * self.res
        self.yc = self.oy + (self.h - 1 - (self.r0 + (self.r1 - self.r0 + 1)
                                          / 2.0)) * self.res
        self.need_fit = False

    def world_to_canvas(self, x, y):
        dx, dy = x - self.xc, y - self.yc
        c, s = math.cos(self.rot), math.sin(self.rot)
        return (self.cw / 2.0 + self.panx + self.s * (dx * c - dy * s),
                self.ch / 2.0 + self.pany - self.s * (dx * s + dy * c))

    def canvas_to_world(self, px, py):
        c, s = math.cos(self.rot), math.sin(self.rot)
        u = (px - self.cw / 2.0 - self.panx) / max(self.s, 1e-6)
        v = -(py - self.ch / 2.0 - self.pany) / max(self.s, 1e-6)
        return (self.xc + u * c + v * s, self.yc - u * s + v * c)

    def zoom_at(self, px, py, factor):
        wx, wy = self.canvas_to_world(px, py)
        self.zoom = max(0.3, min(20.0, self.zoom * factor))
        self._fit(self.cw, self.ch)
        nx, ny = self.world_to_canvas(wx, wy)
        self.panx += px - nx
        self.pany += py - ny
        self._fit(self.cw, self.ch)
        self.dirty = True

    def pan(self, dx, dy):
        self.panx += dx
        self.pany += dy
        self.dirty = True

    def reset_view(self):
        self.zoom = 1.0
        self.panx = self.pany = 0.0
        self.rot = 0.0
        self._rimg = None
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
            # 每帧重算一次取景：_fit 只依赖 zoom/pan/rot，是幂等的，顺便自愈
            # “控件尺寸变了但 Configure 没到”的情况。
            before = (self.s, self.cw, self.ch, self.xc, self.yc)
            self._fit(cw, ch)
            if (self.s, self.cw, self.ch, self.xc, self.yc) != before:
                self.dirty = True
            if not self.dirty and self.photo is not None:
                return self.photo

            pal = self.palette
            W, H = cw * SS, ch * SS
            canvas = Image.new('RGB', (W, H), pal['unknown'])

            # 直接对目标画布做仿射采样，而不是"先把整张栅格缩放再贴"。
            # 后者在高倍放大时会生成一张 30000x18000 的中间图（内存直接爆），
            # 缩放倍率再大，仿射的代价也只跟画布大小有关。
            raster = self._raster_locked()
            coeffs = self._affine(W, H, raster.width, raster.height)
            try:
                canvas = raster.transform((W, H), Image.AFFINE, coeffs,
                                          resample=Image.BILINEAR,
                                          fillcolor=pal['unknown'])
            except TypeError:      # 老 Pillow 没有 fillcolor
                canvas = raster.transform((W, H), Image.AFFINE, coeffs,
                                          resample=Image.BILINEAR)

            draw = ImageDraw.Draw(canvas)
            if trail and len(trail) > 1:
                step = max(1, len(trail) // 1200)
                pts = [self._w2c(x, y) for x, y in trail[::step]]
                draw.line(pts, fill=pal['trail'], width=max(1, SS // 2),
                          joint='curve')
            if plan and len(plan) > 1:
                pts = [self._w2c(x, y) for x, y in plan]
                draw.line(pts, fill=pal['plan'], width=max(1, SS),
                          joint='curve')
            if goal is not None:
                gx, gy = self._w2c(goal[0], goal[1])
                rr = 5 * SS
                draw.ellipse((gx - rr, gy - rr, gx + rr, gy + rr),
                             outline=pal['goal'], width=SS)
            if scan:
                r = max(1.0, 1.1 * SS)
                for x, y in scan:
                    px, py = self._w2c(x, y)
                    if -5 < px < W + 5 and -5 < py < H + 5:
                        draw.ellipse((px - r, py - r, px + r, py + r),
                                     fill=pal['scan'])
            if pose is not None:
                px, py = self._w2c(pose[0], pose[1])
                ln = 10 * SS
                ca, sa = math.cos(pose[2]), math.sin(pose[2])
                draw.line([(px, py), (px + 2.0 * ln * ca, py - 2.0 * ln * sa)],
                          fill=pal['robot'], width=max(1, SS // 2))
                tri = [(px + ln * ca, py - ln * sa),
                       (px - ln * 0.55 * ca + ln * 0.45 * sa,
                        py + ln * 0.55 * sa + ln * 0.45 * ca),
                       (px - ln * 0.55 * ca - ln * 0.45 * sa,
                        py + ln * 0.55 * sa - ln * 0.45 * ca)]
                draw.polygon(tri, fill=pal['robot'], outline=pal['robot_edge'])

            canvas = canvas.resize((cw, ch), Image.BILINEAR)
            self.photo = ImageTk.PhotoImage(canvas)
            self.dirty = False
            return self.photo

    def _affine(self, W, H, rw, rh):
        """Affine coefficients mapping canvas pixel -> raster pixel.

        PIL wants (a, b, c, d, e, f) with
            source_x = a*x + b*y + c
            source_y = d*x + e*y + f
        for every destination pixel (x, y). Deriving it from the same
        world<->canvas transform the overlays use is what keeps the map and
        the laser/robot markers aligned at any rotation.
        """
        def to_raster(px, py):
            u = (px - W / 2.0 - self.panx * SS) / (self.s * SS)
            v = -(py - H / 2.0 - self.pany * SS) / (self.s * SS)
            c, s = math.cos(self.rot), math.sin(self.rot)
            # world offset from the bbox centre
            dx = u * c + v * s
            dy = -u * s + v * c
            # PIL rotates counter-clockwise in display coords (y down), so the
            # forward map is i = (dx*cos - dy*sin), j = -(dx*sin + dy*cos).
            i = rw / 2.0 + (dx * c - dy * s) / self.res
            j = rh / 2.0 - (dx * s + dy * c) / self.res
            return i, j

        i0, j0 = to_raster(0.0, 0.0)
        i1, j1 = to_raster(1.0, 0.0)
        i2, j2 = to_raster(0.0, 1.0)
        return (i1 - i0, i2 - i0, i0, j1 - j0, j2 - j0, j0)

    def _w2c(self, x, y):
        """World -> supersampled canvas pixel (SS applied)."""
        dx, dy = x - self.xc, y - self.yc
        c, s = math.cos(self.rot), math.sin(self.rot)
        sc = self.s * SS
        return (self.cw * SS / 2.0 + self.panx * SS + sc * (dx * c - dy * s),
                self.ch * SS / 2.0 + self.pany * SS - sc * (dx * s + dy * c))

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

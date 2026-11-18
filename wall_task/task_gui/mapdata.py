#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mapdata.py -- read a saved map and answer "is this point clear?".

Kept separate from the GUI so it can be tested without a display: the only
Tk-dependent method is fit(), which builds the ImageTk photo.

The map is exactly what map_server serves: a PGM plus a YAML sidecar.  Values
are 0 = occupied, 205 = unknown, 254 = free; `negate` flips them.
"""
import math
import os

import numpy as np
from PIL import Image


class MapData(object):
    def __init__(self, yaml_path):
        meta = self._read_yaml(yaml_path)
        img_path = meta['image']
        if not os.path.exists(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                    os.path.basename(meta['image']))
        im = Image.open(img_path).convert('L')
        img = np.array(im)
        if meta.get('negate'):
            img = 255 - img
        self.img = img
        self.h, self.w = img.shape
        self.res = meta['resolution']
        self.ox, self.oy = meta['origin'][0], meta['origin'][1]
        # fit the known area, not the whole 40x40 grid
        ys, xs = np.nonzero(img != 205)
        if len(xs) == 0:
            xs = np.array([0])
            ys = np.array([0])
        self.c0, self.c1 = int(xs.min()), int(xs.max())
        self.r0, self.r1 = int(ys.min()), int(ys.max())
        self.zoom = 1.0
        self.cw = self.ch = 0
        self.off_x = self.off_y = 0
        self.canvas_w = self.canvas_h = 0
        self.photo = None

    @staticmethod
    def _read_yaml(path):
        meta = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.0], 'negate': 0,
                'image': ''}
        with open(path) as f:
            for line in f:
                line = line.split('#')[0].strip()
                if not line or ':' not in line:
                    continue
                k, v = line.split(':', 1)
                k, v = k.strip(), v.strip()
                if k == 'image':
                    meta['image'] = v
                elif k == 'resolution':
                    meta['resolution'] = float(v)
                elif k == 'negate':
                    meta['negate'] = int(float(v))
                elif k == 'origin':
                    meta['origin'] = [float(x) for x in
                                      v.strip('[]').split(',')]
        return meta

    # -- drawing -----------------------------------------------------------
    def fit(self, canvas_w, canvas_h, margin=12, max_zoom=4.0):
        """Scale the known area into the canvas; needs a live Tk root."""
        from PIL import ImageTk      # imported here so the module loads headless
        bw = max(self.c1 - self.c0 + 1, 1)
        bh = max(self.r1 - self.r0 + 1, 1)
        self.zoom = max(0.05, min((canvas_w - 2 * margin) / float(bw),
                                  (canvas_h - 2 * margin) / float(bh),
                                  max_zoom))
        self.canvas_w = canvas_w
        self.canvas_h = canvas_h
        self.cw = int(bw * self.zoom)
        self.ch = int(bh * self.zoom)
        # Centre the known area inside a fixed canvas instead of resizing the
        # canvas to the image: a resized canvas leaves dead space below and
        # makes the click maths depend on the window size.
        self.off_x = max(0, (canvas_w - self.cw) // 2)
        self.off_y = max(0, (canvas_h - self.ch) // 2)
        crop = self.img[self.r0:self.r1 + 1, self.c0:self.c1 + 1]
        pil = Image.fromarray(crop).convert('L')
        pil = pil.resize((self.cw, self.ch), Image.NEAREST)
        self.photo = ImageTk.PhotoImage(pil)

    def to_px(self, x, y):
        col = (x - self.ox) / self.res
        row = (self.h - 1 - (y - self.oy) / self.res)
        return (self.off_x + (col - self.c0) * self.zoom,
                self.off_y + (row - self.r0) * self.zoom)

    def to_map(self, px, py):
        col = (px - self.off_x) / self.zoom + self.c0
        row = (py - self.off_y) / self.zoom + self.r0
        return (self.ox + col * self.res,
                self.oy + (self.h - 1 - row) * self.res)

    # -- queries -----------------------------------------------------------
    def clear(self, x, y, radius):
        """True if a disc of `radius` metres around (x, y) has no obstacle."""
        col0 = int(round((x - self.ox) / self.res))
        row0 = int(round((self.h - 1 - (y - self.oy) / self.res)))
        r = int(math.ceil(radius / self.res))
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dr * dr + dc * dc > r * r:
                    continue
                row, col = row0 + dr, col0 + dc
                if not (0 <= row < self.h and 0 <= col < self.w):
                    return False, 'outside the map'
                if self.img[row, col] < 100:
                    return False, 'obstacle within %.2f m' % radius
                if self.img[row, col] < 250:
                    return False, 'unknown space within %.2f m' % radius
        return True, 'clear'

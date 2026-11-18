#!/usr/bin/env python3
"""Find the robot's pose in a saved map from a single laser scan.

AMCL needs an initial pose, and guessing it is a trap: in a rectangular room a
wrong guess either converges to the wrong place or makes a perfectly good map
look broken. This searches the map for the pose that best explains the current
scan, so the initial pose is measured rather than assumed. The result is also
an estimate produced independently of AMCL, which makes it usable as a
cross-check instead of another self-confirmation.

Method: likelihood field.
  * distance transform of the occupancy grid -> distance from any cell to the
    nearest occupied cell
  * for each candidate pose, transform the beam endpoints into the map frame
    and average exp(-d^2 / 2 sigma^2)
  * coarse grid over free cells and 5 deg of heading, then a fine refinement

Beams that land on an occupied cell score 0, not 1: distance is zero there,
which would otherwise make "pose inside a wall" the best possible answer.

Usage:
    python3 coarse_localize.py --map ~/transbot/wall_task/maps/fig8.yaml
    python3 coarse_localize.py --map ... --top 5      show runner-ups
"""

import argparse
import math
import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

try:
    from scipy import ndimage as _ndimage
except ImportError:
    _ndimage = None

# Measured laser mount for this robot. Keep this localization tool standalone;
# the original research-only tb_motion module is not part of this package.
LASER_X, LASER_Y, LASER_YAW = -0.03, 0.0, math.pi


def angle_diff(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def read_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError('%s is not a binary PGM (%r)' % (path, magic))
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if line.startswith(b'#'):
                continue
            tokens += line.split()
        w, h = int(tokens[0]), int(tokens[1])
        data = f.read(w * h)
    return np.frombuffer(data, dtype=np.uint8).reshape(h, w), w, h


def read_yaml(path):
    meta = {}
    with open(path, 'r') as f:
        for line in f:
            if ':' not in line or line.strip().startswith('#'):
                continue
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    origin = meta['origin'].strip('[]').split(',')
    return {
        'image': meta['image'],
        'resolution': float(meta['resolution']),
        'origin': [float(x) for x in origin],
        'negate': int(meta.get('negate', 0)),
    }


def distance_to_occupied(occ, res, max_cells=30):
    """Distance from every cell to the nearest occupied cell, in metres.

    Uses scipy when it is installed and falls back to a chamfer propagation
    otherwise, so the script keeps working on a bare ROS install.

    The fallback is a real compromise, measured on the fig8 map: it
    overestimates distance by 2.3 cm on average (3.9% relative, 9.7% at p95)
    and shifts the likelihood by up to 0.055. That is the same order as the
    margin between two competing poses, so with scipy available the exact
    transform is worth using.
    """
    if _ndimage is not None:
        return _ndimage.distance_transform_edt(~occ) * res

    INF = 1e6
    d = np.where(occ, 0.0, INF).astype(np.float32)
    w1, w2 = 1.0, math.sqrt(2.0)
    neighbours = ((-1, 0, w1), (1, 0, w1), (0, -1, w1), (0, 1, w1),
                  (-1, -1, w2), (-1, 1, w2), (1, -1, w2), (1, 1, w2))
    h, w = d.shape
    # max_cells passes are enough for every cell within that radius to settle:
    # each pass propagates the frontier one cell in all eight directions.
    for _ in range(max_cells):
        for dy, dx, cost in neighbours:
            dst = (slice(max(dy, 0), h + min(dy, 0)),
                   slice(max(dx, 0), w + min(dx, 0)))
            src = (slice(max(-dy, 0), h + min(-dy, 0)),
                   slice(max(-dx, 0), w + min(-dx, 0)))
            sub = d[dst]
            np.minimum(sub, d[src] + cost, out=sub)
    return d * res


class LikelihoodField(object):
    def __init__(self, yaml_path, sigma=0.15):
        meta = read_yaml(yaml_path)
        self.res = meta['resolution']
        self.ox, self.oy = meta['origin'][0], meta['origin'][1]
        img_path = meta['image']
        if not os.path.exists(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                    os.path.basename(meta['image']))
        img, self.w, self.h = read_pgm(img_path)
        if meta['negate']:
            img = 255 - img
        self.occ = img < 65
        self.free = img == 254
        # distance to the nearest occupied cell, in metres, for every cell
        self.dt = distance_to_occupied(self.occ, self.res)
        self.sigma = sigma

    def score(self, px, py, angles, pts):
        """Mean likelihood for one position across many headings.

        pts is (N,2) in base_link. Returns an array over `angles`.
        """
        ca = np.cos(angles)[:, None]
        sa = np.sin(angles)[:, None]
        x = px + pts[None, :, 0] * ca - pts[None, :, 1] * sa
        y = py + pts[None, :, 0] * sa + pts[None, :, 1] * ca
        col = ((x - self.ox) / self.res).astype(np.int32)
        row = (self.h - 1 - (y - self.oy) / self.res).astype(np.int32)
        inside = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        cc = np.clip(col, 0, self.w - 1)
        rr = np.clip(row, 0, self.h - 1)
        d = self.dt[rr, cc]
        # d == 0 means the beam ended inside an obstacle: impossible, score 0
        lik = np.exp(-0.5 * (d / self.sigma) ** 2)
        lik = np.where(inside & (d > 1e-6), lik, 0.0)
        return lik.mean(axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='path to the .yaml')
    ap.add_argument('--angle-step', type=float, default=5.0,
                    help='coarse heading step in degrees (5)')
    ap.add_argument('--pos-step', type=float, default=0.20,
                    help='coarse position step in metres (0.20)')
    ap.add_argument('--beams', type=int, default=180,
                    help='beams to use, evenly subsampled (180)')
    ap.add_argument('--max-range', type=float, default=7.5,
                    help='ignore beams longer than this (7.5)')
    ap.add_argument('--top', type=int, default=3,
                    help='how many distinct candidates to report (3)')
    ap.add_argument('--pose-only', action='store_true',
                    help='print just "x y yaw_rad" and nothing else, for a '
                         'shell script to read')
    args = ap.parse_args()

    field = LikelihoodField(args.map)

    rospy.init_node('coarse_localize', anonymous=True)
    scan = rospy.wait_for_message('/scan', LaserScan, timeout=8)

    r = np.asarray(scan.ranges, dtype=np.float64)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r > 0.25) & (r < args.max_range)
    r, a = r[ok], a[ok]
    if len(r) < 60:
        print('ABORT: only %d usable beams, the scan is too sparse' % len(r))
        return 2

    # laser -> base_link, using this robot's measured mount.
    xl, yl = r * np.cos(a), r * np.sin(a)
    cl, sl = math.cos(LASER_YAW), math.sin(LASER_YAW)
    pts = np.column_stack((xl * cl - yl * sl + LASER_X,
                           xl * sl + yl * cl + LASER_Y))
    step = max(1, len(pts) // args.beams)
    pts = pts[::step]
    if not args.pose_only:
        print('beams used %d of %d   range 0.25-%.1f m'
              % (len(pts), len(r), args.max_range))

    # ---- coarse pass over free cells -------------------------------------
    ys, xs = np.nonzero(field.free)
    sub = max(1, int(round(args.pos_step / field.res)))
    keep = (ys % sub == 0) & (xs % sub == 0)
    ys, xs = ys[keep], xs[keep]
    wx = field.ox + xs * field.res
    wy = field.oy + (field.h - 1 - ys) * field.res
    angles = np.radians(np.arange(0.0, 360.0, args.angle_step))
    if not args.pose_only:
        print('coarse grid: %d positions x %d headings'
              % (len(wx), len(angles)))

    cands = []
    for i in range(len(wx)):
        s = field.score(wx[i], wy[i], angles, pts)
        j = int(s.argmax())
        cands.append((float(s[j]), float(wx[i]), float(wy[i]),
                      float(angles[j])))
    cands.sort(reverse=True)

    # Keep only genuinely different candidates. Position alone is not enough:
    # the same spot with a 1 deg heading change is the same answer, and
    # counting it as a runner-up makes the ambiguity margin look far smaller
    # than it is.
    def different(a, b):
        if math.hypot(a[1] - b[1], a[2] - b[2]) > 0.8:
            return True
        dth = abs(math.degrees(angle_diff(a[3], b[3])))
        return dth > 25.0

    distinct = []
    for c in cands:
        if all(different(c, d) for d in distinct):
            distinct.append(c)
        if len(distinct) >= args.top:
            break

    # ---- fine refinement around the best --------------------------------
    # The window grows if the optimum lands on its edge. The coarse grid is
    # 0.2 m, so a true optimum can easily sit further than the first window:
    # on 2026-11-03 the refinement stopped exactly at -0.30 m and reported a
    # pose 0.3 m off, which looked like a clean answer.
    best = distinct[0]
    da = np.radians(np.arange(-8.0, 8.01, 1.0))
    span = 0.30
    fine = None
    for _ in range(4):
        dxy = np.arange(-span, span + 1e-9, 0.05)
        fine = None
        for dx in dxy:
            for dy in dxy:
                px, py = best[1] + dx, best[2] + dy
                s = field.score(px, py, best[3] + da, pts)
                j = int(s.argmax())
                if fine is None or s[j] > fine[0]:
                    fine = (float(s[j]), float(px), float(py),
                            float(best[3] + da[j]))
        at_edge = (abs(fine[1] - best[1]) >= span - 0.051
                   or abs(fine[2] - best[2]) >= span - 0.051)
        if not at_edge:
            break
        span += 0.20

    s_best, px, py, th = fine
    th = math.atan2(math.sin(th), math.cos(th))
    if args.pose_only:
        print('%.3f %.3f %.4f' % (px, py, th))
        return 0
    print('')
    print('best pose in map frame')
    print('  x = %+.3f m   y = %+.3f m   yaw = %+.1f deg'
          % (px, py, math.degrees(th)))
    print('  likelihood %.3f   (1.0 = every beam exactly on a mapped wall)'
          % s_best)
    print('')
    print('runners-up (check for ambiguity)')
    for c in distinct:
        print('  lik %.3f  at (%+.2f, %+.2f) yaw %+.1f deg'
              % (c[0], c[1], c[2], math.degrees(c[3])))
    if len(distinct) > 1:
        margin = s_best - distinct[1][0]
        print('  margin over the next distinct pose: %.3f  ->  %s'
              % (margin, 'unambiguous' if margin > 0.10
                 else 'AMBIGUOUS, treat the pose with suspicion'))
    print('')
    print('use with AMCL:')
    print('  initial_pose_x:=%.3f initial_pose_y:=%.3f initial_pose_a:=%.4f'
          % (px, py, th))
    return 0


if __name__ == '__main__':
    sys.exit(main())

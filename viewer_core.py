# -*- coding: utf-8 -*-
"""Pure geometry and session association shared by the py2 reader/tests."""
from __future__ import division
import bisect
import glob
import json
import math
import os
import re
import sys


def filesystem_text(path):
    if isinstance(path,bytes): return path.decode(sys.getfilesystemencoding() or 'utf-8')
    return path


def compose(a, b):
    x, y, angle = a
    c, s = math.cos(angle), math.sin(angle)
    return x+c*b[0]-s*b[1], y+s*b[0]+c*b[1], angle+b[2]


def inverse(a):
    x, y, angle = a
    c, s = math.cos(angle), math.sin(angle)
    return -c*x-s*y, s*x-c*y, -angle


def transform_xy(pose, x, y):
    """Apply target<-source once; accepts scalars or numpy arrays."""
    tx, ty, angle = pose
    c, s = math.cos(angle), math.sin(angle)
    return tx+c*x-s*y, ty+s*x+c*y


def world_to_view(x, y, pose, span, pixels):
    """World metres -> pixel inside the cropped map view.

    The view is a `span`-metre square centred on the robot, drawn with the
    robot at the middle and +y pointing up, so the y axis is flipped here.
    Returns None for points outside the square: cropping is what keeps the
    view readable, and a point that falls outside must not be clamped onto
    the border (that would fake a wall at the edge of the window).
    """
    half = span / 2.0
    vx = (x - (pose[0] - half)) / span * pixels
    vy = pixels - (y - (pose[1] - half)) / span * pixels
    if 0 <= vx < pixels and 0 <= vy < pixels:
        return vx, vy
    return None


def session_files(path):
    """同一个会话的所有分卷；单文件包就它自己。

    分卷名是 `<主干>_<YYYYmmdd_HHMMSS>_<序号>.bag`，只有这种形状才按主干去
    glob。以前是 `re.sub(r'_\\d+\\.bag$', '')` 一刀切，单文件包
    `cam_20261111_014312.bag` 会被削成 `cam_20261111`，于是把同一小时里另外
    几个会话一起吞进来 —— 建图会拼出一张不存在的图，阅读器也会读错 TF。
    """
    path = os.path.abspath(filesystem_text(path))
    def split_key(p):
        found = re.match(r'^(.*_\d{8}_\d{6})_(\d+)\.bag$', p)
        return (found.group(1), int(found.group(2))) if found else (p, -1)
    if os.path.isdir(path):
        # 允许直接给会话目录：取第一卷当入口。排除离线建图产生的 .mapping.bag，
        # 它不是原始数据，混进来会把分卷分组搅乱。
        bags = [p for p in sorted(glob.glob(os.path.join(path, '*.bag')), key=split_key)
                if not p.endswith('.mapping.bag')]
        if len(set(split_key(p)[0] for p in bags)) > 1:
            raise ValueError('Directory contains multiple sessions; choose a specific bag')
        if not bags:
            return []
        if bags:
            path = bags[0]
    match = re.match(r'^(.*_\d{8}_\d{6})_\d+\.bag$', path)
    files = sorted(glob.glob(match.group(1)+'_*.bag'), key=split_key) if match else []
    # `stem_*.bag` 也会匹配到 `stem_0.bag.mapping.bag`（离线建图的产物），
    # 它不是原始分卷，混进去会让调用方多读一遍、也可能被当成会话的一部分。
    files = [p for p in files if re.match(re.escape(match.group(1))+r'_\d+\.bag$',p)] if match else []
    return files or ([path] if os.path.isfile(path) else [])


def map_directory(path):
    files=session_files(path)
    if not files:raise ValueError('No source bags')
    stem=os.path.splitext(os.path.basename(files[0]))[0]
    if re.match(r'^.*_\d{8}_\d{6}_\d+$',stem):stem=re.sub(r'_\d+$','',stem)
    return os.path.join(os.path.dirname(files[0]),'maps',stem)


def map_manifest_path(path):
    return os.path.join(map_directory(path),'association.json')


def association(path, manifest=None):
    """Use the current map bundle only; archived maps and loose rasters are not fallbacks."""
    path=filesystem_text(path)
    manifest=filesystem_text(manifest) if manifest else None
    files = session_files(path)
    if not files:
        return None
    if manifest and not os.path.isfile(manifest):
        raise ValueError('Missing map association: '+manifest)
    candidates = [manifest] if manifest else [map_manifest_path(files[0])]
    for candidate in candidates:
        if not os.path.isfile(candidate):
            continue
        with open(candidate) as f:
            data = json.load(f)
        root = os.path.dirname(candidate)
        if set(s['name'] for s in data['sources']) != set(os.path.basename(p) for p in files):
            raise ValueError('Map sources do not match the complete selected session')
        # Identity includes size and mtime; a map belongs to an exact session.
        for source in data['sources']:
            # 用解析出来的第一卷拼路径，不能用入参：入参可能是会话目录，
            # 那样拼出来会变成 <目录的父级>/<卷名>，找不到文件。
            p = os.path.join(os.path.dirname(files[0]), source['name'])
            if os.path.getsize(p) != source['size'] or abs(os.path.getmtime(p)-source['mtime']) > .001:
                raise ValueError('Map association is stale: '+p)
        data['yaml'] = os.path.abspath(os.path.join(root, data['yaml']))
        data['trajectory'] = os.path.abspath(os.path.join(root, data['trajectory'])) if data.get('trajectory') else None
        data['source'] = 'rebuild'
        return data
    return None


def mapped_window(start,end,linked):
    """A newly mapped interval must never borrow the old map's pose outside it."""
    if not linked or not linked.get('replace_map_odom') or not linked.get('record_bounds'):
        return start,end,True,False
    lo,hi=linked['record_bounds']
    a=max(start,lo) if lo is not None else start
    b=min(end,hi) if hi is not None else end
    if b-a<.05:return start,end,False,False
    return a,b,True,(a>start or b<end)


class TransformIndex(object):
    def __init__(self):
        self.dynamic = {}
        self.static = {}
        self.times = {}

    def add(self, parent, child, stamp, pose, static=False):
        key = parent.strip('/'), child.strip('/')
        if static:
            self.static[key] = pose
        else:
            self.dynamic.setdefault(key, []).append((stamp, pose))

    def finish(self):
        for key, values in self.dynamic.items():
            values.sort(key=lambda v: v[0])
            self.times[key] = [v[0] for v in values]

    def edge(self, key, t):
        if key in self.static:
            return self.static[key]
        values = self.dynamic[key]
        i = bisect.bisect_right(self.times[key], t)
        limit = 10.0 if key[0] == 'map' else .5
        if i == 0:
            return values[0][1] if values[0][0]-t <= limit else None
        before = values[i-1]
        if t-before[0] > limit:
            return None
        if i == len(values) or key[0] == 'map':
            return before[1]
        after = values[i]
        if after[0]-t > limit or after[0] == before[0]:
            return before[1]
        u = (t-before[0])/(after[0]-before[0])
        a, b = before[1], after[1]
        da = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
        return a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]), a[2]+u*da

    def lookup(self, target, source, t):
        target, source = target.strip('/'), source.strip('/')
        pending = [(target, (0., 0., 0.))]
        seen = set()
        keys = set(self.dynamic) | set(self.static)
        while pending:
            frame, result = pending.pop(0)
            if frame == source:
                return result
            if frame in seen:
                continue
            seen.add(frame)
            for key in keys:
                if frame not in key:
                    continue
                pose = self.edge(key, t)
                if pose is None:
                    continue
                if key[0] == frame:
                    pending.append((key[1], compose(result, pose)))
                else:
                    pending.append((key[0], compose(result, inverse(pose))))
        return None

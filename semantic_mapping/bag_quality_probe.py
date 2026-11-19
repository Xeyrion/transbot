#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""One bag -> one JSON line of quality metrics.

Runs on the local ROS Python (Melodic py2 / Noetic py3). The GUI (py3)
shells out to this and reads the JSON.

    python2 bag_quality_probe.py <bag>            # fast: index only
    python2 bag_quality_probe.py <bag> --odom     # also walk /odom for distance
    python2 bag_quality_probe.py <bag> --frame /tmp/p.ppm --at 0.5
"""
from __future__ import print_function

import glob
import json
import os
import sys

import numpy as np
import rosbag
import rospy

TOPICS = ['/camera/rgb/image_raw', '/camera/depth/image_raw', '/scan',
          '/odom', '/imu/data']


def stats(ts):
    if len(ts) < 2:
        return None
    ts = sorted(ts)
    gaps = np.diff(ts)
    return {
        'n': len(ts),
        'max_gap': float(gaps.max()),
        'p95_gap': float(np.percentile(gaps, 95)),
        'gt1': int((gaps > 1.0).sum()),
    }


def pair_delta(a_ts, b_ts):
    if not a_ts or not b_ts:
        return None
    a = np.array(sorted(a_ts))
    b = np.array(sorted(b_ts))
    idx = np.clip(np.searchsorted(b, a), 1, len(b) - 1)
    d = np.minimum(np.abs(a - b[idx - 1]), np.abs(a - b[idx]))
    return {
        'median_ms': float(np.median(d) * 1000.0),
        'over50_pct': float((d > 0.05).mean() * 100.0),
    }


def dump_frame(bag, topic, out_path, at=0.5):
    conns = [c for c in bag._connections.values() if c.topic == topic]
    if not conns:
        return False
    entries = list(bag._get_entries(conns))
    if not entries:
        return False
    t0 = entries[0].time.to_sec()
    t1 = entries[-1].time.to_sec()
    t = t0 + (t1 - t0) * at
    try:
        for _, msg, _ in bag.read_messages(topics=[topic],
                                           start_time=rospy.Time.from_sec(t),
                                           end_time=rospy.Time.from_sec(min(t + 1.5, t1))):
            if msg.encoding not in ('bgr8', 'rgb8'):
                return False
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            with open(out_path, 'wb') as f:
                f.write(('P6\n%d %d\n255\n' % (msg.width, msg.height)).encode('ascii'))
                f.write(arr.tobytes())
            return True
    except Exception:
        return False
    return False


def odom_path(bag):
    import math
    dist = 0.0
    prev = None
    for _, msg, _ in bag.read_messages(topics=['/odom']):
        p = msg.pose.pose.position
        if prev is not None:
            dist += math.hypot(p.x - prev[0], p.y - prev[1])
        prev = (p.x, p.y)
    return dist


def main():
    path = sys.argv[1]
    want_odom = '--odom' in sys.argv
    frame = None
    at = 0.5
    if '--frame' in sys.argv:
        frame = sys.argv[sys.argv.index('--frame') + 1]
    if '--at' in sys.argv:
        at = float(sys.argv[sys.argv.index('--at') + 1])

    out = {'bag': os.path.basename(path),
           'size_mb': round(os.path.getsize(path) / 1048576.0, 1)}

    try:
        bag = rosbag.Bag(path, 'r')
    except Exception as e:
        out['err'] = str(e)[:120]
        print(json.dumps(out))
        return

    try:
        dur = bag.get_end_time() - bag.get_start_time()
        out['dur'] = round(dur, 1)
        out['msgs'] = bag.get_message_count()

        conns = {}
        for c in bag._connections.values():
            conns.setdefault(c.topic, []).append(c)

        # Completeness inventory only, not a claim of geometric calibration.
        needed = ['/camera/rgb/camera_info', '/camera/depth/camera_info',
                  '/tf', '/tf_static', '/map', '/cmd_vel']
        out['semantic_inputs'] = {t: sum(1 for _ in bag._get_entries(conns.get(t, [])))
                                  if t in conns else 0 for t in needed}
        out['quality_time_basis'] = 'bag_record_time'
        ts = {}
        for t in TOPICS:
            if t not in conns:
                out[t] = None
                continue
            entries = list(bag._get_entries(conns[t]))
            ts[t] = [e.time.to_sec() for e in entries]
            s = stats(ts[t])
            if s is None:
                out[t] = None
                continue
            s['hz'] = round(s['n'] / dur, 2) if dur else 0
            out[t] = s

        pd = pair_delta(ts.get('/camera/rgb/image_raw'), ts.get('/camera/depth/image_raw'))
        out['pair'] = pd

        if want_odom:
            out['path_m'] = round(odom_path(bag), 2)
        if frame:
            out['frame_ok'] = dump_frame(bag, '/camera/rgb/image_raw', frame, at)
    finally:
        bag.close()

    print(json.dumps(out))


if __name__ == '__main__':
    main()

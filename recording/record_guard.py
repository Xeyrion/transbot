#!/usr/bin/env python3
"""Monotonic duration, disk reserve and wall-clock discontinuity guard."""
import argparse
import glob
import json
import os
import shutil
import time


def running(pid):
    try:
        with open('/proc/%d/stat' % pid) as f:
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return False


def monitor(pid, prefix, duration, min_free_gb, startup=10., clock_limit=.5, health_path=None, diagnostic=False):
    started = time.monotonic()
    writing = None
    wall, mono = time.time(), started
    while running(pid):
        now, real = time.monotonic(), time.time()
        if abs((real-wall)-(now-mono)) > clock_limit:
            return 'clock_jump'
        wall, mono = real, now
        if health_path:
            try:
                with open(health_path) as f: health=json.load(f)
            except (OSError,ValueError): health={}
            if not diagnostic:
                if health.get('state')=='failed': return 'stream_fault'
                if now-started>10. and (now-health.get('updated_monotonic',0)>3. or health.get('state')!='healthy'):
                    return 'health_observer_failed'
        if shutil.disk_usage(os.path.dirname(prefix)).free < min_free_gb * 2**30:
            return 'disk_floor'
        if writing is None and glob.glob(prefix+'*.bag.active'):
            writing = now
        if writing is None and now-started > startup:
            return 'startup_timeout'
        if writing is not None and duration > 0 and now-writing >= duration:
            return 'duration'
        time.sleep(.2)
    if duration > 0 and (writing is None or time.monotonic()-writing < duration-.1):
        return 'early_exit'
    return 'process_exit'


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('pid', type=int); ap.add_argument('prefix')
    ap.add_argument('duration', type=float); ap.add_argument('min_free_gb', type=float)
    ap.add_argument('--health'); ap.add_argument('--diagnostic',action='store_true')
    args = ap.parse_args()
    print(monitor(args.pid, args.prefix, args.duration, args.min_free_gb,
                  health_path=args.health,diagnostic=args.diagnostic), flush=True)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""clock_guard.py -- keep the robot clock within a fraction of a second.

The problem
-----------
Every TF stamp on the wire is written by the robot, but the lookups happen on
this machine (move_base, both costmaps).  The Jetson has no RTC battery, so its
clock drifts against this one.  Once the difference passes transform_tolerance
(1.5 s) every lookup fails with

    Lookup would require extrapolation into the past

and AMCL silently stops publishing /amcl_pose.  That looks exactly like "the
arrow stopped moving", and the usual reaction -- restarting the stack -- throws
away the localisation for a problem that is purely about time.

Why not measure from /scan
--------------------------
The obvious source is a robot-stamped header, but that stamp also carries the
transport latency: 0.4 s on a good link and ~1.5 s while the camera stream is
running.  A guard built on it happily "corrects" a phantom 1 s offset and makes
the real one worse.  So the offset is measured over ssh with round-trip
compensation instead, and this script needs no ROS at all.

Why small steps
---------------
A step smaller than transform_tolerance is invisible to TF: old and new stamps
still overlap.  So the guard corrects as soon as the offset passes --step,
which keeps it far away from the tolerance instead of letting it jump.

Usage
    clock_guard.py --once      # print the current offset and exit
    clock_guard.py --sync      # correct once, verify, exit
    clock_guard.py --dry       # watch and log, never touch the clock
    clock_guard.py             # run in the foreground (Ctrl-C to stop)
"""
import argparse
import os
import re
import subprocess
import sys
import time

DEFAULT_USER = 'jetson'
DEFAULT_PASS = 'yahboom'


def robot_host():
    m = re.match(r'^\w+://([^:/]+)', os.environ.get('ROS_MASTER_URI', ''))
    if m:
        return m.group(1)
    return os.environ.get('ROBOT_IP', '192.168.1.11')


def log(text):
    sys.stdout.write('%s  %s\n' % (time.strftime('%H:%M:%S'), text))
    sys.stdout.flush()


def _ssh(host, user, password, remote, timeout=12):
    env = dict(os.environ, SSHPASS=password)
    cmd = ['sshpass', '-e', 'ssh', '-o', 'StrictHostKeyChecking=no',
           '-o', 'ConnectTimeout=6', '%s@%s' % (user, host), remote]
    return subprocess.run(cmd, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=timeout, env=env)


def probe(host, user, password):
    """(robot - local) in seconds, compensated for the ssh round trip.

    The remote `date` runs roughly halfway through the call, so comparing its
    answer with the midpoint of the call removes most of the latency.
    Returns None when the robot cannot be reached.
    """
    t0 = time.time()
    try:
        r = _ssh(host, user, password, 'date +%s.%N')
    except Exception:
        return None
    t1 = time.time()
    out = r.stdout.decode('utf-8', 'replace').strip().splitlines()
    if r.returncode != 0 or not out:
        return None
    try:
        return float(out[-1]) - (t0 + t1) / 2.0
    except ValueError:
        return None


def sync(host, user, password, tries=3):
    """Set the robot clock to this machine's time.  Returns (ok, residual)."""
    resid = None
    for _ in range(tries):
        t0 = time.time()
        # First pass: "now here".  Later passes subtract the residual left by
        # the previous pass, which is exactly the one-way latency.
        target = t0 if resid is None else t0 - resid
        remote = ("echo %s | sudo -S date -s '@%.3f' >/dev/null 2>&1; "
                  "date +%%s.%%N" % (password, target))
        try:
            r = _ssh(host, user, password, remote)
        except Exception as exc:
            return False, str(exc)
        t1 = time.time()
        out = r.stdout.decode('utf-8', 'replace').strip().splitlines()
        if r.returncode != 0 or not out:
            return False, (r.stderr.decode('utf-8', 'replace').strip()
                           or 'ssh failed')
        try:
            resid = float(out[-1]) - (t0 + t1) / 2.0
        except ValueError:
            return False, 'unexpected reply %r' % out[-1]
        if abs(resid) <= 0.10:
            break
    return True, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--step', type=float, default=0.30,
                    help='correct once the offset passes this many seconds')
    ap.add_argument('--interval', type=float, default=30.0,
                    help='how often to probe')
    ap.add_argument('--warn', type=float, default=2.0,
                    help='above this the TF tree is already broken')
    ap.add_argument('--user', default=DEFAULT_USER)
    ap.add_argument('--password', default=DEFAULT_PASS)
    ap.add_argument('--once', action='store_true')
    ap.add_argument('--sync', action='store_true')
    ap.add_argument('--dry', action='store_true')
    args = ap.parse_args()

    host = robot_host()

    if args.once:
        v = probe(host, args.user, args.password)
        if v is None:
            log('cannot reach %s' % host)
            return 1
        log('offset = %+.3f s' % v)
        return 0

    if args.sync:
        before = probe(host, args.user, args.password)
        if before is None:
            log('cannot reach %s' % host)
            return 1
        ok, res = sync(host, args.user, args.password)
        time.sleep(1.0)
        after = probe(host, args.user, args.password)
        log('before %+.3f s -> after %s  (%s)'
            % (before, 'unreachable' if after is None else '%+.3f s' % after,
               'ok' if ok else res))
        return 0 if ok else 1

    log('watching %s every %.0f s, correcting past %.2f s'
        % (host, args.interval, args.step))
    while True:
        v = probe(host, args.user, args.password)
        if v is None:
            log('cannot reach %s' % host)
        elif abs(v) > args.step:
            if args.dry:
                log('offset %+.3f s (dry run)' % v)
            else:
                if abs(v) > args.warn:
                    log('offset %+.2f s -- TF is already broken; syncing, but '
                        'the stack needs a restart' % v)
                ok, res = sync(host, args.user, args.password)
                if ok:
                    log('offset %+.2f s -> corrected, residual %+.2f s'
                        % (v, res))
                else:
                    log('correct failed: %s' % res)
        time.sleep(max(5.0, args.interval))


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass

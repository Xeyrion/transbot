#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Record what the navigation stack decided, and why it looked like that.

The point is not to log raw topics -- rosbag already does that.  The point is
to turn them into a timeline of *behaviours* with a reason attached:

    16:20:31.4 - 16:20:34.8   spin    车头与路径夹角 47°，原地对齐
    16:20:34.8 - 16:20:36.1   drive   沿路径前进
    16:20:36.1 - 16:20:36.9   reverse 目标在背后，倒车比掉头便宜

That answers "为什么它突然倒车/转圈" without guessing from memory.

Usage
-----
    ./nav_trace.py                      # record until Ctrl-C
    ./nav_trace.py --duration 60        # fixed 60 s
    ./nav_trace.py --out ~/traces/run1  # where to write (default: timestamped)
    ./nav_trace.py --selftest           # no ROS: run the classifier on
                                        # synthetic samples, check the labels

Writes trace.csv (raw, 10 Hz) and report.md (timeline + summary) into --out.
"""

from __future__ import print_function

import argparse
import bisect
import csv
import math
import os
import signal
import sys
import threading
import time

# ---------------------------------------------------------------- geometry

def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def project(points, x, y):
    """Closest point on a polyline.

    Returns (dist, s, tangent) where s is the arc length from the start and
    tangent is the direction of travel there.  None if the path is degenerate.
    """
    if not points or len(points) < 2:
        return None
    best = None
    s_acc = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            continue
        t = ((x - ax) * dx + (y - ay) * dy) / (seg * seg)
        t = max(0.0, min(1.0, t))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(x - px, y - py)
        if best is None or d < best[0]:
            best = (d, s_acc + t * seg, math.atan2(dy, dx))
        s_acc += seg
    return best


def path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


# ---------------------------------------------------------------- recorder

class Recorder(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.cmd = []       # (t, v, w)
        self.odom = []      # (t, x, y, yaw, v, w)
        self.amcl = []      # (t, x, y, yaw, sx, sy, syaw)
        self.status = []    # (t, code, text)
        self.plan = []      # (t, [(x, y), ...])
        self.local = []     # (t, [(x, y), ...])
        self.t0 = time.time()

    # -- callbacks -----------------------------------------------------
    def cb_cmd(self, msg):
        t = time.time()
        with self.lock:
            self.cmd.append((t, msg.linear.x, msg.angular.z))

    def cb_odom(self, msg):
        t = msg.header.stamp.to_sec()
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.odom.append((t, msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw,
                              msg.twist.twist.linear.x,
                              msg.twist.twist.angular.z))

    def cb_amcl(self, msg):
        t = msg.header.stamp.to_sec()
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c = msg.pose.covariance
        with self.lock:
            self.amcl.append((t, msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw,
                              math.sqrt(max(c[0], 0.0)),
                              math.sqrt(max(c[7], 0.0)),
                              math.degrees(math.sqrt(max(c[35], 0.0)))))

    def cb_status(self, msg):
        if not msg.status_list:
            return
        st = msg.status_list[-1]
        with self.lock:
            self.status.append((time.time(), st.status, st.text))

    def cb_plan(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self.lock:
            self.plan.append((time.time(), pts))
            del self.plan[:-40]      # keep the recent history only

    def cb_local(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self.lock:
            self.local.append((time.time(), pts))
            del self.local[:-40]

    # -- helpers -------------------------------------------------------
    @staticmethod
    def _last(series, t, max_age):
        """Last sample at or before t, or None if older than max_age."""
        if not series:
            return None
        ts = [s[0] for s in series]
        i = bisect.bisect_right(ts, t) - 1
        if i < 0:
            return None
        if t - series[i][0] > max_age:
            return None
        return series[i]

    def timeline(self, step=0.1):
        with self.lock:
            cmd = list(self.cmd)
            odom = list(self.odom)
            amcl = list(self.amcl)
            status = list(self.status)
            plan = list(self.plan)
        if not cmd and not odom:
            return []
        # Start from whichever stream began first: a recording started before
        # the goal is clicked has odom but no /cmd_vel yet, and those seconds
        # of "waiting" are part of the story.
        t_start = min(cmd[0][0] if cmd else 1e18, odom[0][0] if odom else 1e18)
        t_end = max(cmd[-1][0] if cmd else 0.0, odom[-1][0] if odom else 0.0)
        rows = []
        t = t_start
        while t <= t_end:
            c = self._last(cmd, t, 0.4)
            o = self._last(odom, t, 0.6)
            a = self._last(amcl, t, 2.5)
            s = self._last(status, t, 5.0)
            p = self._last(plan, t, 3.0)
            rows.append({
                't': t,
                'v_cmd': None if c is None else c[1],
                'w_cmd': None if c is None else c[2],
                'odom': o,
                'amcl': a,
                'status': s,
                'plan': p[1] if p else None,
            })
            t += step
        return rows


# ---------------------------------------------------------------- analysis

STATUS = {0: 'pending', 1: 'active', 2: 'preempted', 3: 'succeeded',
          4: 'aborted', 5: 'rejected', 6: 'preempting', 7: 'recalling',
          8: 'recalled', 9: 'lost'}


def classify(rows, xy_tol=0.12):
    """Attach behaviour + reason to every row."""
    out = []
    flip_hist = []
    prev_sign = 0
    for r in rows:
        v, w = r['v_cmd'], r['w_cmd']
        o, a, p = r['odom'], r['amcl'], r['plan']
        label = 'idle'
        reason = ''

        if v is None or w is None:
            label, reason = 'idle', '没有 /cmd_vel'
            out.append(dict(r, label=label, reason=reason))
            continue

        # --- what the controller asked for
        if abs(v) < 0.02 and abs(w) < 0.05:
            label = 'stop'
        elif v < -0.02:
            label = 'reverse'
        elif abs(v) < 0.02:
            label = 'spin'
        else:
            label = 'drive'

        # --- context
        goal_d = None
        head_err = None
        off_path = None
        if p and a is not None:
            gx, gy = p[-1]
            goal_d = math.hypot(gx - a[1], gy - a[2])
            pr = project(p, a[1], a[2])
            if pr:
                off_path, _, tangent = pr
                head_err = wrap(a[3] - tangent)
        if p and a is None and o is not None:
            gx, gy = p[-1]
            goal_d = math.hypot(gx - o[1], gy - o[2])

        if label == 'spin':
            if goal_d is not None and goal_d < xy_tol + 0.05 and abs(w) > 0.08:
                reason = '到点了，原地转到目标朝向（latch）'
            elif head_err is not None and abs(head_err) > 0.35:
                reason = '车头与路径差 %.0f°，先原地对齐' % math.degrees(head_err)
            elif off_path is not None and off_path > 0.30:
                reason = '离路径 %.2f m，先转向再走' % off_path
            else:
                reason = '原地调整朝向'
        elif label == 'reverse':
            if head_err is not None and abs(head_err) > 2.2:
                reason = '路径在背后，倒车比掉头便宜'
            elif goal_d is not None and goal_d < 0.35:
                reason = '快到目标，倒车微调位置'
            else:
                reason = '回退修正（min_vel_x = -0.10 允许倒车）'
        elif label == 'drive':
            if head_err is not None and abs(head_err) > 0.25:
                reason = '边走边纠偏 %.0f°' % math.degrees(head_err)
            else:
                reason = '沿路径前进'
        elif label == 'stop':
            st = r['status']
            if st and st[1] == 3:
                reason = '目标完成'
            elif st and st[1] == 4:
                reason = 'move_base 放弃（%s）' % (st[2] or 'aborted')
            elif st and st[1] == 2:
                reason = '目标被取消'
            else:
                reason = '停住'

        # --- limit cycle: sign of w flipping faster than the chassis can act
        sgn = 0
        if abs(w) > 0.05:
            sgn = 1 if w > 0 else -1
        flip_hist.append((r['t'], sgn))
        while flip_hist and r['t'] - flip_hist[0][0] > 1.0:
            flip_hist.pop(0)
        flips = sum(1 for (_, s1), (_, s2) in zip(flip_hist, flip_hist[1:])
                    if s1 and s2 and s1 != s2)
        if flips >= 4 and label in ('drive', 'spin'):
            reason += '；ω 1 秒换向 %d 次（极限环）' % flips

        # --- commanded vs achieved
        if o is not None and label in ('drive', 'reverse'):
            if abs(v) > 0.05 and abs(o[4]) < abs(v) * 0.35:
                reason += '；发了 %.2f 实走 %.2f（底盘没跟上）' % (v, o[4])

        out.append(dict(r, label=label, reason=reason, goal_d=goal_d,
                        head_err=head_err, off_path=off_path))
    return out


def segments(rows, min_len=0.4):
    """Merge consecutive rows with the same label into segments."""
    if not rows:
        return []
    segs = []
    cur = {'label': rows[0]['label'], 'start': rows[0]['t'],
           'end': rows[0]['t'], 'reasons': [rows[0]['reason']]}
    for r in rows[1:]:
        if r['label'] == cur['label']:
            cur['end'] = r['t']
            cur['reasons'].append(r['reason'])
        else:
            segs.append(cur)
            cur = {'label': r['label'], 'start': r['t'], 'end': r['t'],
                   'reasons': [r['reason']]}
    segs.append(cur)
    merged = []
    for s in segs:
        if merged and s['end'] - s['start'] < min_len:
            merged[-1]['end'] = s['end']
            merged[-1]['reasons'].extend(s['reasons'])
        else:
            merged.append(s)
    for s in merged:
        counts = {}
        for x in s['reasons']:
            counts[x] = counts.get(x, 0) + 1
        s['reason'] = max(counts.items(), key=lambda kv: kv[1])[0]
    return merged


def anomalies(rows):
    """Localisation corrections: AMCL moved more than the robot actually did.

    Comparing consecutive AMCL poses is meaningless while the robot drives --
    at 0.2 m/s the pose legitimately changes 10 cm between updates, and the
    first version of this function reported 181 "jumps" for a run where AMCL
    was in fact healthy.  The honest test is AMCL's delta against odometry's
    delta over the same interval.
    """
    jumps = []
    prev = None
    for r in rows:
        a, o = r['amcl'], r['odom']
        if a is None or o is None:
            continue
        if prev is not None:
            da = math.hypot(a[1] - prev[0][1], a[2] - prev[0][2])
            do = math.hypot(o[1] - prev[1][1], o[2] - prev[1][2])
            dya = abs(math.degrees(wrap(a[3] - prev[0][3])))
            dyo = abs(math.degrees(wrap(o[3] - prev[1][3])))
            # Yaw needs a looser gate than position: AMCL answers with the
            # pose of the scan it just processed, which is ~0.5 s old, so on
            # a turning robot the two deltas differ by w*0.5 -- about 6 deg
            # at 0.2 rad/s.  That is lag, not a correction.
            if abs(da - do) > 0.05 or abs(dya - dyo) > 15.0:
                jumps.append((r['t'], da - do, dya - dyo))
        prev = (a, o)
    return jumps


def tracking(rows):
    """How faithfully the base executed what was asked.

    This is the number that explains most bad gait on this chassis: the
    angular ratio sits near 1.4, so every turn is 40 % stronger than the
    planner predicted and every correction overshoots.
    """
    out = {}
    for name, key, lim in (('linear', 'v_cmd', 0.05), ('angular', 'w_cmd', 0.10)):
        pairs = [(r[key], r['odom'][4 if name == 'linear' else 5])
                 for r in rows
                 if r.get(key) is not None and r['odom'] is not None
                 and abs(r[key]) > lim]
        if not pairs:
            continue
        ratios = sorted(b / a for a, b in pairs)
        out[name] = (len(ratios), ratios[len(ratios) // 2],
                     ratios[len(ratios) // 10], ratios[len(ratios) * 9 // 10])
    cmd = [r for r in rows if r.get('v_cmd') is not None]
    if cmd:
        n = float(len(cmd))
        out['sat_fwd'] = 100.0 * sum(1 for r in cmd if r['v_cmd'] >= 0.199) / n
        out['sat_back'] = 100.0 * sum(1 for r in cmd if r['v_cmd'] <= -0.099) / n
        out['sat_turn'] = 100.0 * sum(1 for r in cmd if abs(r['w_cmd']) >= 0.279) / n
    return out


# ---------------------------------------------------------------- report

LABEL_ZH = {'drive': '前进', 'reverse': '倒车', 'spin': '原地转',
            'stop': '停', 'idle': '无指令'}


def write_report(rows, segs, jumps, out_dir, started):
    dur = rows[-1]['t'] - rows[0]['t'] if rows else 0.0
    share = {}
    for r in rows:
        share[r['label']] = share.get(r['label'], 0.0) + 0.1

    os.makedirs(out_dir)
    with open(os.path.join(out_dir, 'trace.csv'), 'w') as f:
        w = csv.writer(f)
        w.writerow(['t_rel', 'label', 'v_cmd', 'w_cmd', 'v_odom', 'w_odom',
                    'x', 'y', 'yaw_deg', 'sigma_xy', 'goal_d', 'head_err_deg',
                    'off_path', 'reason'])
        t0 = rows[0]['t'] if rows else 0.0
        for r in rows:
            o, a = r['odom'], r['amcl']
            w.writerow([
                '%.2f' % (r['t'] - t0), r['label'],
                '' if r['v_cmd'] is None else '%.3f' % r['v_cmd'],
                '' if r['w_cmd'] is None else '%.3f' % r['w_cmd'],
                '' if o is None else '%.3f' % o[4],
                '' if o is None else '%.3f' % o[5],
                '' if a is None else '%.2f' % a[1],
                '' if a is None else '%.2f' % a[2],
                '' if a is None else '%.1f' % math.degrees(a[3]),
                '' if a is None else '%.3f' % math.hypot(a[4], a[5]),
                '' if r.get('goal_d') is None else '%.2f' % r['goal_d'],
                '' if r.get('head_err') is None else
                '%.1f' % math.degrees(r['head_err']),
                '' if r.get('off_path') is None else '%.2f' % r['off_path'],
                r['reason']])

    lines = []
    lines.append('# 导航记录 %s' % time.strftime('%Y-%m-%d %H:%M:%S',
                                                time.localtime(started)))
    lines.append('')
    lines.append('时长 %.1f s，%d 个采样点（10 Hz）。' % (dur, len(rows)))
    lines.append('')
    lines.append('## 时间分布')
    lines.append('')
    lines.append('| 行为 | 时长 | 占比 |')
    lines.append('|---|---|---|')
    # Divide by the sum of the bins, not by (last-first): the first sample
    # has no width, so the two differ by one 0.1 s tick and the shares used
    # to add up to 102 %.
    total = sum(share.values()) or 0.1
    for k in sorted(share, key=lambda x: -share[x]):
        lines.append('| %s | %.1f s | %.0f%% |'
                     % (LABEL_ZH.get(k, k), share[k],
                        100.0 * share[k] / total))
    lines.append('')
    lines.append('## 时间线')
    lines.append('')
    lines.append('| 起 | 止 | 秒 | 行为 | 原因 |')
    lines.append('|---|---|---|---|---|')
    t0 = rows[0]['t'] if rows else 0.0
    for s in segs:
        lines.append('| %6.1f | %6.1f | %4.1f | %s | %s |'
                     % (s['start'] - t0, s['end'] - t0,
                        s['end'] - s['start'],
                        LABEL_ZH.get(s['label'], s['label']),
                        s['reason']))
    lines.append('')
    tr = tracking(rows)
    lines.append('## 指令 vs 实际')
    lines.append('')
    if 'linear' in tr:
        n, med, p10, p90 = tr['linear']
        lines.append('- 线速度 实测/指令：中位 **%.2f**（p10 %.2f，p90 %.2f，n=%d）'
                     % (med, p10, p90, n))
    if 'angular' in tr:
        n, med, p10, p90 = tr['angular']
        lines.append('- 角速度 实测/指令：中位 **%.2f**（p10 %.2f，p90 %.2f，n=%d）'
                     % (med, p10, p90, n))
    if 'sat_turn' in tr:
        lines.append('- 指令打在限幅上：前进 %.0f%% / 倒车 %.0f%% / 转向 %.0f%%'
                     % (tr['sat_fwd'], tr['sat_back'], tr['sat_turn']))
    lines.append('')
    lines.append('## 异常')
    lines.append('')
    if jumps:
        lines.append('AMCL 修正 %d 次（相对里程计 >5 cm 或 >15°）：' % len(jumps))
        for t, dd, dy in jumps[:10]:
            lines.append('- %6.1f s  位置差 %+.2f m / 角度差 %+.1f°'
                         % (t - t0, dd, dy))
    else:
        lines.append('AMCL 与里程计一致，没有异常修正。')
    lines.append('')

    text = '\n'.join(lines) + '\n'
    with open(os.path.join(out_dir, 'report.md'), 'w') as f:
        f.write(text)
    return text


# ---------------------------------------------------------------- selftest

def selftest():
    """Feed synthetic samples through classify() and check the labels."""
    t = 0.0
    rows = []

    def add(n, v, w, yaw, goal=(5.0, 0.0), path=((0.0, 0.0), (5.0, 0.0)),
            odom_v=None, status=(1, 'active')):
        nonlocal t
        for _ in range(n):
            rows.append({
                't': t, 'v_cmd': v, 'w_cmd': w,
                'odom': (t, 0.0, 0.0, yaw, v if odom_v is None else odom_v, w),
                'amcl': (t, 0.0, 0.0, yaw, 0.05, 0.05, 2.0),
                'status': (t, status[0], status[1]),
                'plan': list(path)})
            t += 0.1

    add(10, 0.15, 0.0, 0.0)                 # straight ahead
    add(10, 0.0, 0.28, 0.9)                 # facing 51 deg off, spinning
    add(10, -0.08, 0.0, 3.0)                # path behind -> reverse
    add(10, 0.0, 0.20, 0.0,
        path=((0.0, 0.0), (0.05, 0.0)))        # at goal, rotate
    add(10, 0.0, 0.0, 0.0, status=(3, ''))  # done

    out = classify(rows)
    got = [r['label'] for r in out[::10]]
    want = ['drive', 'spin', 'reverse', 'spin', 'stop']
    ok = got == want
    for i, r in enumerate(out[::10]):
        print('%-8s %s' % (r['label'], r['reason']))
    print('labels', got)
    if not ok:
        print('FAIL: expected', want)
        return 1
    segs = segments(out)
    print('segments', len(segs))
    print('SELFTEST PASSED')
    return 0


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='0 = record until Ctrl-C')
    ap.add_argument('--xy-tol', type=float, default=0.12)
    ap.add_argument('--selftest', action='store_true')
    # ROS remaps (`/cmd_vel:=/test_cmd`) arrive in sys.argv too; ignore them
    # here, rospy picks them up on its own.
    args, _unknown = ap.parse_known_args()

    if args.selftest:
        return selftest()

    import rospy
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, Path
    from actionlib_msgs.msg import GoalStatusArray

    rec = Recorder()
    rospy.init_node('nav_trace', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, rec.cb_cmd, queue_size=50)
    rospy.Subscriber('/odom', Odometry, rec.cb_odom, queue_size=50)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, rec.cb_amcl,
                     queue_size=20)
    rospy.Subscriber('/move_base/status', GoalStatusArray, rec.cb_status,
                     queue_size=20)
    rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, rec.cb_plan,
                     queue_size=5)
    rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, rec.cb_local,
                     queue_size=5)

    stop = threading.Event()

    def on_sigint(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    print('recording; Ctrl-C to stop')
    started = time.time()
    deadline = started + args.duration if args.duration > 0 else None
    while not stop.is_set() and not rospy.is_shutdown():
        if deadline and time.time() >= deadline:
            break
        time.sleep(0.2)

    out_dir = args.out or os.path.join(
        os.path.expanduser('~/.local/share/transbot/wall_task/traces'),
        time.strftime('%Y%m%d-%H%M%S', time.localtime(started)))
    out_dir = os.path.expanduser(out_dir)
    rows = classify(rec.timeline(), xy_tol=args.xy_tol)
    if not rows:
        print('no /cmd_vel and no /odom -- is the stack running?')
        return 1
    segs = segments(rows)
    jumps = anomalies(rows)
    text = write_report(rows, segs, jumps, out_dir, started)
    print('wrote %s' % out_dir)
    print('')
    print('\n'.join(text.splitlines()[:24]))
    return 0


if __name__ == '__main__':
    sys.exit(main())

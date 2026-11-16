#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_probe.py -- 机器人上的“状态探针”。

作用
    每隔 1 秒往 stdout 打一行 JSON，描述机器人此刻的真实状态：
    在不在录、bag 多大、磁盘还剩多少、RGB/深度/雷达多少 Hz、
    底盘驱动还在不在、ROS master 还在不在。

为什么用“探针 + SSH”而不是直接订阅 ROS 话题
    1. 图像话题 20 MB/s，控制机走 WiFi 根本收不动，订阅了只会更卡；
    2. ROS 挂掉的时候，恰恰是最需要看状态的时候。探针不依赖 ROS 也能跑
       （ROS 部分失败就报 ros_ok=0，其余字段照常输出）。

运行方式（由 record_gui.py 自动启动，一般不用手敲）
    bash -lc 'source /opt/ros/melodic/setup.bash; export ROS_IP=192.168.1.11; \
              exec python3 -u /home/jetson/record_probe.py'

也可以手动看一眼：
    ssh jetson@192.168.1.11 'python3 -u /home/jetson/record_probe.py' | head -5
"""

import glob
import json
import os
import re
import subprocess
import sys
import time

TOPICS = ['/camera/rgb/image_raw', '/camera/depth/image_raw', '/scan']
PERIOD = 1.0

counts = {}
ros_ok = False
ros_error = ''


# ---------------------------------------------------------------- 小工具
def sh(cmd):
    """跑一条 shell 命令，返回去空白后的 stdout；失败返回空串。"""
    try:
        out = subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL,timeout=2)
        return out.decode('utf-8', 'replace').strip()
    except Exception:
        return ''


def read_file(path, limit=4096):
    try:
        with open(path, 'rb') as f:
            data = f.read()
        return data[-limit:].decode('utf-8', 'replace').strip()
    except Exception:
        return ''


def disk_free_gb(path=None):
    try:
        path=path or os.path.expanduser('~/bags')
        st = os.statvfs(path if os.path.isdir(path) else os.path.expanduser('~'))
        return st.f_bavail * st.f_frsize / 1e9
    except Exception:
        return -1.0


def bag_state():
    """找到正在录的 rosbag，返回 (路径前缀, 文件列表, 已录秒数)。

    rosbag record 的命令行长这样：
        python2 .../rosbag record -O /home/jetson/bags/cam_20261110_022211.bag --lz4 ...
    正在写的文件叫 cam_20261110_022211_0.bag.active，收尾后改名去掉 .active。
    """
    pids = sh('pgrep -f "bin/rosbag record"').split()
    if not pids:
        return '', [], 0.0, 0

    # 以前拿到第一个 -O 就 break，双录制时第二个会话完全看不见，
    # 所以“已经在录两份”这件事只能等盘满了才发现。这里全部收集。
    prefixes = []
    for pid in pids:
        try:
            with open('/proc/%s/cmdline' % pid, 'rb') as f:
                argv = f.read().split(b'\0')
        except Exception:
            continue
        argv = [a.decode('utf-8', 'replace') for a in argv if a]
        for i, a in enumerate(argv):
            if a == '-O' and i + 1 < len(argv):
                prefixes.append(argv[i + 1])
                break

    if not prefixes:
        return '', [], 0.0, 0
    prefix = prefixes[0]

    base = prefix[:-4] if prefix.endswith('.bag') else prefix
    files = glob.glob(base + '*.bag') + glob.glob(base + '*.bag.active')
    files = sorted(set(os.path.realpath(p) for p in files))

    elapsed = 0.0
    m = re.search(r'(\d{8})_(\d{6})', os.path.basename(prefix))
    if m:
        try:
            started = time.mktime(time.strptime(m.group(1) + m.group(2), '%Y%m%d%H%M%S'))
            elapsed = max(0.0, time.time() - started)
        except Exception:
            elapsed = 0.0
    if elapsed <= 0 and files:
        elapsed = max(0.0, time.time() - min(os.path.getmtime(p) for p in files))

    # 所有会话的分卷都算进来，否则 mb/mbps 只统计了第一路
    for other in prefixes[1:]:
        ob = other[:-4] if other.endswith('.bag') else other
        files += sorted(set(os.path.realpath(p) for p in
                            glob.glob(ob + '*.bag') + glob.glob(ob + '*.bag.active')))
    return prefix, files, elapsed, len(prefixes)


def topic_rates(prev, dt):
    """用两次采样之间的计数差算频率。"""
    out = {}
    if not ros_ok or dt <= 0:
        return {t: -1.0 for t in TOPICS}
    for t in TOPICS:
        out[t] = round(max(0,counts.get(t, 0) - prev.get(t, 0)) / dt, 1)
    return out


# ---------------------------------------------------------------- 主循环
def main():
    global counts,ros_ok,ros_error

    prev_counts = {}
    prev_t = time.monotonic()
    prev_bytes = 0
    prev_bytes_t = time.monotonic()
    prev_prefix=None
    no_bag_ticks = 0

    while True:
        now = time.monotonic()
        dt = now - prev_t
        prefix, files, elapsed, sessions = bag_state()
        health={}
        if prefix:
            stem=prefix[:-4] if prefix.endswith('.bag') else prefix
            try:
                with open(stem+'.health.json') as f: health=json.load(f)
            except (OSError,ValueError): pass
        fresh=-.2<=now-health.get('updated_monotonic',0)<3
        counts={t:v['count'] for t,v in health.get('streams',{}).items()} if fresh else {}
        ros_ok=fresh and health.get('state')=='healthy'
        ros_error=','.join(health.get('faults',[])) or ('observer unavailable' if not fresh else '')
        if prefix!=prev_prefix: prev_counts=dict(counts); prev_bytes=0; prev_prefix=prefix
        if health.get('started_monotonic') is not None: elapsed=max(0,now-health['started_monotonic'])

        rates = topic_rates(prev_counts, dt)
        prev_counts = dict(counts)
        prev_t = now

        total = sum(os.path.getsize(p) for p in files if os.path.exists(p)) if files else 0

        # 写盘速率：只在同一个 bag 内算，换文件就重新开始
        if total and prev_bytes and total >= prev_bytes:
            dt2 = max(0.2, now - prev_bytes_t)
            mbps = round((total - prev_bytes) / 1e6 / dt2, 1)
        else:
            mbps = 0.0
        prev_bytes, prev_bytes_t = total, now

        if files:
            no_bag_ticks = 0
        else:
            no_bag_ticks += 1

        log_tail = read_file('/tmp/record_full_console.log', 600).splitlines()
        log_last = log_tail[-1].strip() if log_tail else ''

        rec = {
            't': round(time.time(), 1),
            'rec': 1 if files else 0,
            'bag': os.path.basename(prefix) if prefix else '',
            'files': len(files),
            'sessions': sessions,
            'mb': round(total / 1e6, 1),
            'elapsed': round(elapsed, 1),
            'mbps': mbps,
            'disk_free_gb': round(disk_free_gb(os.path.dirname(prefix) if prefix else None), 1),
            'health_state':health.get('state','observer_unavailable' if prefix else 'not_recording'),
            'health_faults':health.get('faults',[]),
            'rgb': rates.get('/camera/rgb/image_raw', -1.0),
            'depth': rates.get('/camera/depth/image_raw', -1.0),
            'scan': rates.get('/scan', -1.0),
            'driver': 1 if sh('pgrep -f "transbot_driver.py"') else 0,
            'master': 1 if sh('pgrep -f "bin/rosmaster"') else 0,
            'ros_ok': 1 if ros_ok else 0,
            'ros_error': ros_error,
            'log': log_last[:160],
        }
        sys.stdout.write(json.dumps(rec, ensure_ascii=False) + '\n')
        sys.stdout.flush()
        time.sleep(PERIOD)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        sys.stderr.write('record_probe crashed: %s\n' % exc)
        sys.exit(1)

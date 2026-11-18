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
import atexit
import json
import os
import re
import subprocess
import sys
import time
import threading
from stop_recording import processes
from session_status import phase_path,last_result_path

TOPICS = ['/camera/rgb/image_raw', '/camera/depth/image_raw', '/scan']
PERIOD = 1.0

# ---------------------------------------------------------------- 小工具
class SourceStatus:
    """Background ROS counters; sensor startup never blocks the SSH heartbeat."""
    def __init__(self):
        self.proc=None;self.master=None;self.data={};self.lock=threading.Lock()
        self.next_start=0.

    def update(self, master):
        if master!=self.master or (self.proc and self.proc.poll() is not None):
            self.close();self.master=master
        if master and self.proc is None and time.monotonic()>=self.next_start:
            helper=os.path.join(os.path.dirname(os.path.abspath(__file__)),'live_health.py')
            try:
                proc=subprocess.Popen(['python2',helper,'full','-','--rates-only'],
                    stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,
                    universal_newlines=True,bufsize=1)
                self.proc=proc
                threading.Thread(target=self.read,args=(proc,),daemon=True).start()
            except OSError:
                self.next_start=time.monotonic()+3.

    def read(self,proc):
        try:
            for line in proc.stdout:
                try:data=json.loads(line)
                except ValueError:continue
                with self.lock:
                    if self.proc is proc:self.data=data
        finally:
            proc.stdout.close();proc.wait()

    def snapshot(self):
        with self.lock:return dict(self.data)

    def close(self):
        proc=self.proc;self.proc=None
        with self.lock:self.data={}
        if proc and proc.poll() is None:proc.terminate()


def read_file(path, limit=4096):
    try:
        with open(path, 'rb') as f:
            f.seek(0,os.SEEK_END)
            f.seek(max(0,f.tell()-limit))
            data = f.read(limit)
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


def file_size(path):
    try:return os.path.getsize(path)
    except OSError:return 0


def bag_state(procs=None):
    """找到正在录的 rosbag，返回 (路径前缀, 文件列表, 已录秒数)。

    rosbag record 的命令行长这样：
        python2 .../rosbag record -O /home/jetson/bags/cam_20261110_022211.bag --lz4 ...
    正在写的文件叫 cam_20261110_022211_0.bag.active，收尾后改名去掉 .active。
    """
    prefixes=[]
    for argv in (processes() if procs is None else procs).values():
        scripts={os.path.basename(a) for a in argv[:2]}
        recorder=bool(argv) and (argv[0].endswith('/rosbag/record') or ('rosbag' in scripts and 'record' in argv))
        if recorder and '-O' in argv:
            index=argv.index('-O')
            if index+1<len(argv) and argv[index+1] not in prefixes:prefixes.append(argv[index+1])
    if not prefixes:return '',[],0.,0
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


def topic_rates(snapshot, now):
    if not (0<=now-snapshot.get('updated_monotonic',0)<3.):
        return {t:-1. for t in TOPICS}
    result={}
    for topic in TOPICS:
        item=snapshot.get('streams',{}).get(topic,{})
        if now-item.get('last_monotonic',0)>3.:
            result[topic]=0.
        else:
            result[topic]=snapshot.get('rates',{}).get(topic,-1.)
    return result


def recording_phase(procs,files,phase_info):
    wrappers={pid for pid,args in procs.items() if any(os.path.basename(a) in ('record_full.sh','auto_record_cam.sh') for a in args[:2])}
    phase=phase_info.get('phase') if phase_info.get('owner_pid') in wrappers else None
    if phase=='finalizing':return 'finalizing'
    if files:return 'recording'
    return 'preparing' if wrappers else 'idle'


# ---------------------------------------------------------------- 主循环
def main():
    prev_bytes = 0
    prev_bytes_t = time.monotonic()
    prev_prefix=None
    source=SourceStatus()
    atexit.register(source.close)

    while True:
        procs=processes()
        master=tuple(sorted(pid for pid,args in procs.items() if any(os.path.basename(a)=='rosmaster' for a in args[:2])))
        source.update(master)
        prefix, files, elapsed, sessions = bag_state(procs)
        now=time.monotonic()
        try:phase_info=json.loads(read_file(phase_path()))
        except ValueError:phase_info={}
        health={}
        if prefix:
            stem=prefix[:-4] if prefix.endswith('.bag') else prefix
            try:
                live_path=phase_info.get('health_path') or stem+'.health.json'
                with open(live_path) as f: health=json.load(f)
            except (OSError,ValueError): pass
        snapshot=source.snapshot()
        now=time.monotonic()
        fresh=0<=now-snapshot.get('updated_monotonic',0)<3.
        ros_ok=fresh
        ros_error='' if fresh else ('source observer starting' if master else 'ROS master unavailable')
        if prefix!=prev_prefix:prev_bytes=0;prev_prefix=prefix
        rates=topic_rates(snapshot,now)

        total = sum(file_size(p) for p in files)

        # 写盘速率：只在同一个 bag 内算，换文件就重新开始
        if total and prev_bytes and total >= prev_bytes:
            dt2 = max(0.2, now - prev_bytes_t)
            mbps = round((total - prev_bytes) / 1e6 / dt2, 1)
        else:
            mbps = 0.0
        prev_bytes, prev_bytes_t = total, now

        log_tail = read_file('/tmp/record_full_console.log', 600).splitlines()
        log_last = log_tail[-1].strip() if log_tail else ''
        try:last_recording=json.loads(read_file(last_result_path()))
        except ValueError:last_recording={}

        rec = {
            't': round(time.time(), 1),
            'rec': 1 if files else 0,
            'phase': recording_phase(procs,files,phase_info),
            'last_recording': last_recording,
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
            'driver': int(any(any(os.path.basename(a)=='transbot_driver.py' for a in args[:2]) for args in procs.values())),
            'master': int(bool(master)),
            'sensor_state': 'live' if fresh else 'starting' if master else 'unavailable',
            'ros_ok': 1 if ros_ok else 0,
            'ros_error': ros_error,
            'log': log_last[:160],
        }
        sys.stdout.write(json.dumps(rec, ensure_ascii=False) + '\n')
        sys.stdout.flush()
        time.sleep(max(.05,PERIOD-(time.monotonic()-now)))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        sys.stderr.write('record_probe crashed: %s\n' % exc)
        sys.exit(1)

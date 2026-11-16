#!/usr/bin/env python3
"""Read recording state and paths from actual process arguments."""
import datetime
import glob
import os
import shutil

from stop_recording import processes


def snapshot(procs):
    prefixes = set()
    starting = []
    for pid, args in procs.items():
        if any(os.path.basename(a) in ('record_full.sh','auto_record_cam.sh') for a in args[:2]):
            out = args[args.index('-o')+1] if '-o' in args else '/home/jetson/bags'
            starting.append((pid, out))
        is_recorder = (args and (args[0].endswith('/rosbag/record') or
                       any(os.path.basename(a) == 'rosbag' for a in args[:2])))
        if is_recorder and '-O' in args:
            prefixes.add(args[args.index('-O')+1])
    files = []
    for prefix in prefixes:
        stem = prefix[:-4] if prefix.endswith('.bag') else prefix
        files += glob.glob(glob.escape(stem)+'*.bag')
        files += glob.glob(glob.escape(stem)+'*.bag.active')
    return dict(state='recording' if prefixes else ('starting' if starting else 'idle'),
                directories=sorted(set(os.path.dirname(p) for p in prefixes) |
                                   set(d for _, d in starting)),
                files=sorted(set(files)), sessions=len(prefixes))


def main():
    state = snapshot(processes())
    print('时间：'+str(datetime.datetime.now()))
    print('状态：'+{'recording': '录制运行中', 'starting': '正在准备数据源（可取消）', 'idle': '未在录制'}[state['state']])
    for d in state['directories'] or ['/home/jetson/bags']:
        if os.path.isdir(d):
            print('目录：%s；可用 %.2f GiB' % (d, shutil.disk_usage(d).free/2**30))
    if state['sessions']:
        print('会话数：%d；本次分卷合计 %.1f MiB' %
              (state['sessions'], sum(os.path.getsize(p) for p in state['files'])/2**20))
        for p in state['files']:
            print('  %.1f MiB  %s' % (os.path.getsize(p)/2**20, p))
    print('日志尾部：')
    try:
        with open('/tmp/record_full_console.log') as f:
            print(''.join(f.readlines()[-6:]).rstrip())
    except OSError:
        pass


if __name__ == '__main__':
    main()

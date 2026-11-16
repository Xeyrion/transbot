#!/usr/bin/env python3
"""Cancel startup and let rosbag finish. Never SIGKILL a recorder."""
import os
import signal
import time


def processes():
    result = {}
    for name in os.listdir('/proc'):
        if not name.isdigit() or int(name) == os.getpid():
            continue
        try:
            with open('/proc/%s/cmdline' % name, 'rb') as f:
                args = [x.decode('utf-8', 'replace') for x in f.read().split(b'\0') if x]
            result[int(name)] = args
        except (IOError, OSError):
            pass
    return result


def groups():
    wrappers, recorders = [], []
    for pid, args in processes().items():
        if any(os.path.basename(a) in ('record_full.sh','auto_record_cam.sh') for a in args[:2]):
            wrappers.append(pid)
        # The Python rosbag front end spawns this executable. Signal the actual
        # recorder, not its Python parent (Melodic's signal handler is faulty).
        if args and (args[0].endswith('/rosbag/record') or
                     (os.path.basename(args[0]) == 'record' and '-O' in args)):
            recorders.append(pid)
    return wrappers, recorders


def parent_of(pid):
    try:
        with open('/proc/%d/stat'%pid) as f: return int(f.read().rsplit(')',1)[1].split()[1])
    except (OSError,ValueError): return 0


def managed(pid,wrappers,parent=parent_of):
    seen=set()
    while pid>1 and pid not in seen:
        seen.add(pid); pid=parent(pid)
        if pid in wrappers: return True
    return False


def main():
    wrappers, recorders = groups()
    print('取消会话 %s；收尾录制进程 %s' % (wrappers, recorders), flush=True)
    for pid in wrappers:
        if managed(pid,set(wrappers)): continue
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    for pid in recorders:
        if managed(pid,set(wrappers)): continue
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        if not any(groups()):
            print('会话已停止。', flush=True)
            return 0
        time.sleep(.2)
    print('ERROR: 仍有进程在收尾，未强杀；请检查 .active 和日志。', flush=True)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())

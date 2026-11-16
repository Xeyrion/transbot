#!/usr/bin/env python3
"""Validated JSON startup request. No shell interpolation of paths or labels."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


def active_recorders():
    result=[]
    for name in os.listdir('/proc'):
        if not name.isdigit() or int(name)==os.getpid(): continue
        try: argv=[x.decode('utf-8','replace') for x in Path('/proc',name,'cmdline').read_bytes().split(b'\0') if x]
        except OSError: continue
        if not argv: continue
        scripts={os.path.basename(s) for s in argv[:2]}
        if scripts & {'record_full.sh','auto_record_cam.sh'} or (argv[0].endswith('/rosbag/record') and '-O' in argv) or ('rosbag' in scripts and 'record' in argv):
            result.append(int(name))
    return result


def start(payload):
    if payload.get('protocol_version')!=1: raise ValueError('Unsupported recorder protocol')
    root=Path(payload['root']).resolve()
    bad=[]
    for name,digest in payload['files'].items():
        if Path(name).name!=name: raise ValueError('Invalid bundle name')
        target=root/name
        if not target.is_file() or hashlib.sha256(target.read_bytes()).hexdigest()!=digest: bad.append(name)
    if bad: raise ValueError('Recorder bundle mismatch: '+', '.join(bad)+'. Deploy the full bundle before recording.')
    active=active_recorders()
    if active: raise RuntimeError('Recording/finalization is already active: '+str(active))
    duration=payload['duration']
    if not isinstance(duration,int) or isinstance(duration,bool) or duration<0: raise ValueError('Invalid duration')
    cmd=['/bin/bash',str(root/'record_full.sh'),'-d',str(duration),'-o',payload['out']]
    if payload.get('name'): cmd+=['-n',payload['name']]
    with open('/tmp/record_full_console.log','ab') as log:
        child=subprocess.Popen(cmd,cwd=str(root),stdin=subprocess.DEVNULL,stdout=log,stderr=log,
            start_new_session=True,preexec_fn=lambda:signal.signal(signal.SIGHUP,signal.SIG_IGN))
    return dict(started=True,pid=child.pid,bundle_verified=True)


if __name__=='__main__':
    try:
        if '--check-idle' in sys.argv:
            active=active_recorders(); print(json.dumps(dict(active_recorders=active))); raise SystemExit(bool(active))
        print(json.dumps(start(json.load(sys.stdin))))
    except Exception as exc:
        print(str(exc),file=sys.stderr); raise SystemExit(1)

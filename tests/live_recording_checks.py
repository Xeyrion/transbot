#!/usr/bin/env python3
"""Private ROS master + real recorder + liveness fault injection, no robot."""
import glob
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client
import shutil

root=Path(sys.argv[1]).resolve(); children=[]
with socket.socket() as s: s.bind(('127.0.0.1',0)); port=s.getsockname()[1]
env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',MIN_FREE_GB='0',STREAM_TIMEOUT_SEC='1')
env.pop('ROS_HOSTNAME',None)
with tempfile.TemporaryDirectory(prefix='bagtools-live-') as d:
    tmp=Path(d)
    full_fixture=tmp/'full_fixture'; full_fixture.mkdir()
    for path in (root/'recording').glob('*'):
        if path.suffix in ('.py','.sh'):
            target=full_fixture/path.name; shutil.copy2(str(path),str(target)); target.chmod(0o755)
    shutil.copy2(str(root/'calibration_schema.py'),str(full_fixture/'calibration_schema.py'))
    full=full_fixture/'record_full.sh'
    full.write_text(full.read_text().replace('/tmp/transbot_record.',str(tmp/'lock.')))
    def start(cmd):
        p=subprocess.Popen(cmd,env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        children.append(p); return p
    try:
        start(['roscore','-p',str(port)])
        deadline=time.monotonic()+20
        while True:
            try: xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/check'); break
            except OSError:
                if time.monotonic()>deadline: raise
                time.sleep(.2)
        drop=tmp/'drop'
        start(['python2',str(root/'tests/live_fixture_publish.py'),str(drop)])
        time.sleep(2)
        results=[]
        for name,inject in [('normal',False),('depth_stall',True),('full_wrapper_cancel',False)]:
            if drop.exists(): drop.unlink()
            folder=tmp/name; log=open(str(tmp/(name+'.log')),'wb')
            cmd=['bash',str(root/'recording/auto_record_cam.sh'),'-p','full','-d','4' if not inject else '15','-o',str(folder)]
            if name=='full_wrapper_cancel': cmd=['bash',str(full),'--no-launch','-d','15','-o',str(folder)]
            p=subprocess.Popen(cmd,
                               env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            children.append(p)
            if inject or name=='full_wrapper_cancel':
                deadline=time.monotonic()+90
                while not list(folder.glob('session_*/*.bag.active')):
                    if p.poll() is not None: raise RuntimeError('Recorder exited before injection: '+(tmp/(name+'.log')).read_text())
                    if time.monotonic()>deadline: raise RuntimeError('No active bag in '+name+': '+(tmp/(name+'.log')).read_text()[-6000:])
                    time.sleep(.1)
                time.sleep(2)
                if inject: drop.write_text('/camera/depth/image_raw')
                else: p.terminate()
            p.wait(timeout=50); log.close()
            paths=list(folder.glob('session_*/*.session.json')); assert len(paths)==1,(name,(tmp/(name+'.log')).read_text())
            meta=json.loads(paths[0].read_text())
            assert meta['complete']==(not inject),(name,meta,(tmp/(name+'.log')).read_text())
            if inject: assert meta['stop_reason']=='stream_fault',meta
            results.append(dict(case=name,passed=True,manifest=meta))
        print(json.dumps(results,indent=2))
    finally:
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=10)

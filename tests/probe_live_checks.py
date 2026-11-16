#!/usr/bin/env python3
"""Over one minute of idle status; live data, missing topic, recovery, no bag."""
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

root=Path(sys.argv[1]).resolve();children=[]
with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1')
env.pop('ROS_HOSTNAME',None)
def start(cmd,pipe=False):
    p=subprocess.Popen(cmd,env=env,stdout=subprocess.PIPE if pipe else subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL,start_new_session=True)
    children.append(p);return p

with tempfile.TemporaryDirectory(prefix='bagtools-probe-') as tmp:
    drop=Path(tmp)/'drop'
    try:
        start(['roscore','-p',str(port)])
        deadline=time.monotonic()+20
        while True:
            try:xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/test');break
            except OSError:
                if time.monotonic()>deadline:raise
                time.sleep(.1)
        start(['python2',str(root/'tests/live_fixture_publish.py'),str(drop)])
        probe=start(['python3','-u',str(root/'recording/record_probe.py')],True)
        started=time.monotonic();rows=[];last=None;gaps=[]
        while time.monotonic()-started<72.:
            if not select.select([probe.stdout],[],[],3.)[0]:raise AssertionError('Status heartbeat stopped')
            line=probe.stdout.readline();assert line,probe.poll()
            now=time.monotonic();d=json.loads(line.decode());elapsed=now-started
            assert not d['rec'] and d['health_state']=='not_recording',d
            if last is not None:gaps.append(now-last)
            last=now;rows.append((elapsed,d))
            if 22.<elapsed<32.:drop.write_text('/scan')
            elif drop.exists():drop.unlink()
        assert any(10<t<20 and d['rgb']>10 and d['depth']>10 and d['scan']>10 for t,d in rows)
        assert any(27<t<32 and d['scan']==0 and d['rgb']>10 for t,d in rows)
        assert any(t>45 and d['scan']>10 for t,d in rows)
        assert max(gaps)<2.5,max(gaps)
        print(json.dumps(dict(state='passed',duration_s=rows[-1][0],messages=len(rows),max_heartbeat_gap_s=max(gaps),
                              checks=['idle_rates','missing_topic_zero','topic_recovery','continuous_heartbeat_over_one_minute','no_recording']),indent=2))
    finally:
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=8)

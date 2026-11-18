#!/usr/bin/env python3
"""Private ROS master: configured TF, wrong TF, duplicate source and missing TF."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

root=Path(sys.argv[1]).resolve()
config=root/'recording/calibration.json'
data=json.loads(config.read_text());item=data['transforms'][0]
args=[str(v) for v in item['xyz']+item['quaternion_xyzw']]+[item['parent'],item['child']]
with socket.socket() as sock:
    sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1')
env.pop('ROS_HOSTNAME',None)
children=[];results=[]

def start(cmd):
    child=subprocess.Popen(cmd,env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    children.append(child);return child

def stop(child):
    if child.poll() is None:
        os.killpg(child.pid,signal.SIGINT);child.wait(timeout=10)

def check(name,match):
    p=subprocess.run(['python2',str(root/'recording/check_tf.py'),'--calibration',str(config)],
                     env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=10)
    assert p.stdout.strip(),(name,p.returncode,p.stderr.decode())
    result=json.loads(p.stdout.decode())
    assert result['matched']==match and (p.returncode==0)==match,(name,result,p.stderr)
    results.append(dict(case=name,passed=True,check=result))

try:
    start(['roscore','-p',str(port)])
    deadline=time.monotonic()+20
    while True:
        try:
            xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/test');break
        except OSError:
            if time.monotonic()>deadline:raise
            time.sleep(.1)
    primary=start(['rosrun','tf2_ros','static_transform_publisher']+args+['__name:=configured_camera'])
    time.sleep(1)
    check('matching_configuration',True)
    wrong=start(['rosrun','tf','static_transform_publisher','0.08','0','0.10403','-0.05','0','0',
                 'base_link','camera_link','30','__name:=legacy_camera'])
    time.sleep(.5);check('legacy_dynamic_conflict',False);stop(wrong)
    duplicate=start(['rosrun','tf2_ros','static_transform_publisher']+args+['__name:=duplicate_camera'])
    time.sleep(.5);check('identical_duplicate_publishers',False);stop(duplicate)
    check('single_publisher_restored',True)
    stop(primary);check('missing_installation',False)
    print(json.dumps(results,indent=2))
finally:
    for child in reversed(children):stop(child)

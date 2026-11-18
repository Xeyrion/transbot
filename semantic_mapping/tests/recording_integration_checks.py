#!/usr/bin/env python3
"""Valid ROS bag fixtures behind command stubs; no master or hardware access."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import shutil

ROOT=Path(sys.argv[1]).resolve()


def executable(path,text):
    path.write_text(text); path.chmod(0o755)


with tempfile.TemporaryDirectory(prefix='bagtools-recording-') as d:
    tmp=Path(d); (tmp/'bin').mkdir(); (tmp/'home').mkdir()
    fixture=tmp/'recorder'; fixture.mkdir()
    shutil.copy2(str(ROOT/'calibration_schema.py'),str(fixture/'calibration_schema.py'))
    for path in (ROOT/'recording').glob('*'):
        if path.suffix in ('.py','.sh'): shutil.copy2(str(path),str(fixture/path.name))
    shutil.copy2(str(fixture/'live_health.py'),str(fixture/'real_live_health.py'))
    (fixture/'live_health.py').write_text('''import json,os,sys,time
from real_live_health import monotonic
while True:
 with open(sys.argv[2]+'.tmp','w') as f: json.dump(dict(state='healthy',faults=[],updated_monotonic=monotonic()),f)
 os.rename(sys.argv[2]+'.tmp',sys.argv[2]); time.sleep(.05)
''')
    executable(tmp/'bin/rostopic', '#!/bin/sh\n[ "$1" = list ] && printf "/scan\\n/odom\\n/tf\\n"; exit 0\n')
    executable(tmp/'bin/rosnode', '#!/bin/sh\nexit 0\n')
    executable(tmp/'bin/rosbag', '''#!/usr/bin/env python2
import os,signal,sys,time
import rosbag,rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
stop=[False]
signal.signal(signal.SIGINT,lambda *a:stop.__setitem__(0,True))
prefix=sys.argv[sys.argv.index('-O')+1][:-4]+'_0.bag'
b=rosbag.Bag(prefix+'.active','w')
for name,msg in [('/scan',LaserScan()),('/odom',Odometry()),('/tf',TFMessage())]:
 if name!=os.environ.get('DROP_TOPIC'): b.write(name,msg,rospy.Time.from_sec(10))
if 'FAKE_EXIT' not in os.environ:
 while not stop[0]: time.sleep(.02)
b.close(); os.rename(prefix+'.active',prefix)
sys.exit(int(os.environ.get('FAKE_EXIT','0')))
''')
    base=dict(os.environ,HOME=str(tmp/'home'),PATH=str(tmp/'bin')+os.pathsep+os.environ['PATH'],
              ROS_IP='127.0.0.1',ROS_MASTER_URI='http://127.0.0.1:1',MIN_FREE_GB='0')
    results=[]
    for name,extra,duration,expected in [
        ('timed_normal_stop',{},'1',True),
        ('failed_child_valid_data',{'FAKE_EXIT':'7'},'0',False),
        ('missing_recorded_tf',{'FAKE_EXIT':'0','DROP_TOPIC':'/tf'},'0',False),
        ('unexpected_early_exit',{'FAKE_EXIT':'0'},'5',False)]:
        folder=tmp/name
        p=subprocess.run(['bash',str(fixture/'auto_record_cam.sh'),'-p','nav','-d',duration,'-o',str(folder)],
            env=dict(base,**extra),stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=20)
        files=list(folder.glob('session_*/*.session.json')); assert len(files)==1,(name,p.stderr)
        data=json.loads(files[0].read_text())
        assert data['complete']==expected,(name,data,p.stdout,p.stderr)
        assert (p.returncode==0)==expected,(name,p.returncode)
        if name=='timed_normal_stop': assert data['stop_reason']=='duration',data
        if name=='failed_child_valid_data': assert data['data_quality']['status']=='passed',data
        results.append(dict(name=name,passed=True,returncode=p.returncode,manifest=data))
    print(json.dumps(results,indent=2))

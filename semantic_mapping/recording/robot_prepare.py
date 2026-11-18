#!/usr/bin/env python3
"""GUI maintenance RPC: shared recording lock, fresh control-PC time, one ROS stack."""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import select
import shlex
import signal
import subprocess
import sys
import time

from remote_start import active_recorders,verify_bundle
from stop_recording import processes
from record_guard import running
from publish_calibration import commands

ACTIONS=('clock','camera','lidar','all')
TOPICS=['/camera/rgb/image_raw','/camera/depth/image_raw','/scan','/odom']
PREPARATION_LOCK='/tmp/transbot_record.lock'


def emit(kind,message='',**data):
    print(json.dumps(dict(data,event=kind,message=message)),flush=True)


def valid_utc(value):
    if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value):
        raise ValueError('Invalid control-PC time')
    if not 2024<=datetime.datetime.fromtimestamp(value,datetime.timezone.utc).year<2100:
        raise ValueError('Control-PC clock is outside the supported date range')
    return float(value)


def read_json():
    if not select.select([sys.stdin],[],[],20.)[0]:raise RuntimeError('Control-PC reply timed out')
    line=sys.stdin.readline()
    if not line:raise RuntimeError('Control connection closed')
    return json.loads(line)


class Preparation:
    def __init__(self,root,ask_time):
        self.root=Path(root);self.ask_time=ask_time
        setup='source /opt/ros/melodic/setup.bash; '
        for path in [Path.home()/'software/transbot_library/devel/setup.bash',Path.home()/'transbot_ws/devel/setup.bash']:
            if path.exists():setup+='source '+shlex.quote(str(path))+' --extend; '
        raw=subprocess.check_output(['bash','-c',setup+'env -0'],timeout=12)
        self.env=dict(x.decode().split('=',1) for x in raw.split(b'\0') if b'=' in x)
        self.env.update(ROS_MASTER_URI='http://localhost:11311',ROS_IP=os.environ.get('ROS_IP','192.168.1.11'))
        self.env.pop('ROS_HOSTNAME',None)

    def run(self,args,timeout=20,check=True,logs=False):
        p=subprocess.run(args,env=self.env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=timeout)
        output=p.stdout.decode('utf-8','replace').strip()
        if logs and output:emit('log',output[-12000:])
        if check and p.returncode:raise RuntimeError(output[-1500:] or 'Command failed: '+str(args[0]))
        return p

    def data_ready(self,topics):
        code="""import rospy,time,sys
from rospy.msg import AnyMsg
rospy.init_node('bagtools_prepare_probe',anonymous=True,disable_signals=True)
topics=%r
seen=set()
def callback(msg,topic):seen.add(topic)
subs=[rospy.Subscriber(t,AnyMsg,callback,callback_args=t,queue_size=1,buff_size=2**22) for t in topics]
deadline=time.time()+3
while time.time()<deadline and len(seen)<len(topics):time.sleep(.05)
sys.exit(0 if len(seen)==len(topics) else 1)
"""%topics
        return self.run(['python2','-c',code],timeout=10,check=False).returncode==0

    def stop_sources(self):
        emit('phase','Stopping managed ROS data sources before recovery')
        targets=[]
        for pid,args in processes().items():
            if not any(os.path.basename(x)=='roslaunch' for x in args[:2]):continue
            known=any(os.path.basename(x)=='record_sources.launch' for x in args[2:])
            known=known or any(package in args and launch in args for package,launch in [
                ('transbot_nav','transbot_record.launch'),('transbot_program','Transbot_Program.launch'),
                ('astra_camera','astrapro.launch'),('rplidar_ros','rplidar.launch')])
            if known:
                targets.append(pid)
                try:os.kill(pid,signal.SIGTERM)
                except ProcessLookupError:pass
        deadline=time.monotonic()+20.
        while any(running(pid) for pid in targets) and time.monotonic()<deadline:time.sleep(.1)
        if any(running(pid) for pid in targets):raise RuntimeError('ROS data-source shutdown timed out')
        p=self.run(['rosnode','list'],timeout=6,check=False)
        if p.returncode==0:
            remaining=[n for n in p.stdout.decode().splitlines() if n!='/rosout' and
                       not n.startswith(('/bagtools_source_status_','/bagtools_prepare_probe_',
                                         '/bagtools_tf_check_','/record_gui_teleop_'))]
            if remaining:raise RuntimeError('Other ROS nodes remain; stop their launch before changing time: '+', '.join(remaining))

    def launch_sources(self):
        unused,cmds=commands(str(self.root/'calibration.json'))
        camera=[cmd for cmd in cmds if cmd[-2:]==['base_link','camera_link']]
        if len(camera)!=1:raise ValueError('Camera installation transform is required')
        emit('phase','Starting the calibrated camera, lidar and base data sources')
        with open('/tmp/transbot_record.log','ab') as log:
            subprocess.Popen(['roslaunch',str(self.root/'record_sources.launch'),
                              'camera_transform:='+' '.join(camera[0][3:])],env=self.env,
                             stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True,close_fds=True)
        deadline=time.monotonic()+60.
        while time.monotonic()<deadline:
            if self.data_ready(TOPICS):break
            time.sleep(.5)
        else:raise RuntimeError('Sensor startup timed out; see /tmp/transbot_record.log')
        self.preflight(check=True)

    def preflight(self,check=False):
        emit('phase','Checking data, clock and installation transforms')
        return self.run(['bash',str(self.root/'preflight.sh')],timeout=35,check=check,logs=True).returncode==0

    def synchronize(self):
        reference=valid_utc(self.ask_time())
        offset=reference-time.time()
        emit('log','Control-PC minus robot clock: %.3f s'%offset)
        if abs(offset)<=.2:
            self.run(['python3',str(self.root/'clock_check.py'),'--seconds','1'],timeout=8)
            return False
        self.stop_sources()
        # A second handshake is essential: stopping ROS can take many seconds.
        reference=valid_utc(self.ask_time())
        emit('phase','Synchronizing UTC from the control PC with ROS data sources stopped')
        self.run(['sudo','-n','date','-u','-s','@%.6f'%reference],timeout=5,logs=True)
        for device in ['/dev/rtc0','/dev/rtc1']:
            if Path(device).exists():
                try:
                    p=self.run(['sudo','-n','hwclock','--systohc','--utc','--rtc',device],timeout=4,check=False)
                    if p.returncode:emit('log','RTC write failed: '+device)
                except subprocess.TimeoutExpired:emit('log','RTC write timed out: '+device)
        self.run(['python3',str(self.root/'clock_check.py'),'--seconds','1'],timeout=8)
        self.launch_sources()
        return True

    def execute(self,action):
        if action not in ACTIONS:raise ValueError('Unknown preparation action')
        if action in ('clock','all'):
            restarted=self.synchronize()
            if action=='clock' or restarted:return
        else:
            self.run(['python3',str(self.root/'clock_check.py'),'--seconds','1'],timeout=8)
            wanted=TOPICS[:2] if action=='camera' else ['/scan']
            if self.data_ready(wanted):
                emit('phase','Requested sensor is already publishing; no restart needed')
                return
        if self.preflight():return
        # The vendor lidar has respawn=true. Recover its owner launch, not a
        # competing second lidar node; camera recovery follows the same ownership.
        self.stop_sources();self.launch_sources()


def serve():
    import fcntl
    payload=read_json();root=verify_bundle(payload)
    if root!=Path(__file__).resolve().parent:raise ValueError('Preparation root differs from installed helper')
    with open(PREPARATION_LOCK,'a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Recording or preparation is active; wait or stop it first')
        active=active_recorders()
        if active:raise RuntimeError('Recording/finalization is active: '+str(active))
        def ask_time():
            emit('clock_request')
            return read_json()['utc']
        Preparation(root,ask_time).execute(payload['action'])
        emit('done','Preparation completed; sensor status will refresh automatically')


def client(target,password,root,action):
    from recorder_bundle import bundle
    valid_utc(time.time())
    argv=(['sshpass','-p',password] if password else [])+[
        'ssh','-T','-o','ConnectTimeout=6','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3',target,
        'exec python3 -u '+shlex.quote(root+'/robot_prepare.py')+' --serve']
    p=subprocess.Popen(argv,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=sys.stderr,universal_newlines=True,bufsize=1)
    try:
        p.stdin.write(json.dumps(dict(protocol_version=1,files=bundle(),root=root,action=action))+'\n');p.stdin.flush()
        done=False
        for line in p.stdout:
            event=json.loads(line)
            if event['event']=='clock_request':
                p.stdin.write(json.dumps(dict(utc=valid_utc(time.time())))+'\n');p.stdin.flush()
            else:
                if event.get('message'):print(event['message'],flush=True)
                if event['event']=='done':done=True
        p.wait()
        return 0 if p.returncode==0 and done else 1
    finally:
        p.stdin.close();p.stdout.close()
        if p.poll() is None:p.terminate();p.wait(timeout=10)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--serve',action='store_true')
    parser.add_argument('--action',choices=ACTIONS,default='all');args=parser.parse_args()
    try:
        if args.serve:serve()
        else:raise SystemExit(client(os.environ.get('ROBOT','jetson@192.168.1.11'),os.environ.get('ROBOT_PASS','yahboom'),
                                   os.environ.get('ROBOT_HOME','/home/jetson'),args.action))
    except Exception as exc:
        if args.serve:emit('error',str(exc))
        else:print(str(exc),file=sys.stderr)
        raise SystemExit(1)

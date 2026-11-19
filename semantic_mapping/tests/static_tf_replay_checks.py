#!/usr/bin/env python2
"""Verify late ROS subscribers receive every static edge after navigation repair."""
from __future__ import print_function
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
try:
    import xmlrpclib
except ImportError:
    import xmlrpc.client as xmlrpclib
import rosbag
import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

root=os.path.abspath(sys.argv[1]); port=int(sys.argv[2]) if len(sys.argv)>2 else 11429
probe=socket.socket()
try:probe.bind(('127.0.0.1',port))
finally:probe.close()
os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%port
os.environ['ROS_IP']='127.0.0.1'; os.environ.pop('ROS_HOSTNAME',None)
scratch=tempfile.mkdtemp(prefix='bagtools-static-replay-'); children=[]
log=open(os.path.join(scratch,'ros.log'),'w')
def start(argv):
    p=subprocess.Popen(argv,stdout=log,stderr=log,preexec_fn=os.setsid)
    children.append(p); return p
try:
    source=os.path.join(scratch,'cam_20261115_120000_0.bag')
    with rosbag.Bag(source,'w') as bag:
        for parent,child in [('base_footprint','base_link'),('base_link','laser'),
                             ('camera_link','camera_depth_optical_frame')]:
            tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child
            tr.transform.rotation.w=1.
            bag.write('/tf_static',TFMessage([tr]),rospy.Time.from_sec(10.))
        for i in range(5):
            stamp=rospy.Time.from_sec(10.1+i*.1)
            odom=Odometry();odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser'
            scan.angle_increment=.1;scan.range_min=.1;scan.range_max=10.;scan.ranges=[2.]*20
            tr=TransformStamped();tr.header.frame_id='odom';tr.child_frame_id='base_footprint'
            tr.header.stamp=stamp;tr.transform.rotation.w=1.
            for topic,msg in [('/scan',scan),('/odom',odom),('/tf',TFMessage([tr]))]:
                bag.write(topic,msg,stamp)
    repaired=os.path.join(scratch,'repaired')
    subprocess.check_call([sys.executable,os.path.join(root,'semantic','prepare_navigation.py'),source,repaired,
                           '--allow-estimated-extrinsics'])
    start(['roscore','-p',str(port)])
    master=xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI']);deadline=time.time()+20
    while True:
        try:master.getPid('/static_replay_test');break
        except Exception:
            if time.time()>deadline:raise RuntimeError('Private master did not start')
            time.sleep(.1)
    rospy.init_node('late_static_replay_test',anonymous=True,disable_signals=True)
    start(['rosbag','play','--clock','--keep-alive',os.path.join(repaired,os.path.basename(source))])
    time.sleep(3)  # Subscribe after playback has reached its final message.
    msg=rospy.wait_for_message('/tf_static',TFMessage,timeout=5)
    edges={(t.header.frame_id.strip('/'),t.child_frame_id.strip('/')) for t in msg.transforms}
    expected={('base_footprint','base_link'),('base_link','laser'),('base_link','camera_link'),
              ('camera_link','camera_depth_optical_frame')}
    assert expected<=edges, (expected,edges)
    import tf2_ros
    buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
    deadline=time.time()+5
    while True:
        try:
            buffer.lookup_transform('base_footprint','laser',rospy.Time(0))
            buffer.lookup_transform('base_footprint','camera_depth_optical_frame',rospy.Time(0))
            break
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            if time.time()>deadline:raise
            time.sleep(.05)
    print(json.dumps(dict(state='passed',checks=['late_latched_complete_tree','late_tf2_navigation_chain',
                                                'late_tf2_camera_chain']),indent=2))
finally:
    rospy.signal_shutdown('test complete')
    for p in reversed(children):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            deadline=time.time()+10
            while p.poll() is None and time.time()<deadline:time.sleep(.1)
            if p.poll() is None:os.killpg(p.pid,signal.SIGKILL)
            p.wait()
    log.close();shutil.rmtree(scratch)

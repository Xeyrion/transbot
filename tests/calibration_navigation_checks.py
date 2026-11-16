#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Offline fixtures: depth intrinsics, stationary evidence, candidate export, TF repair."""
from __future__ import print_function
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import CameraInfo,Image,LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

ROOT=os.path.abspath(sys.argv[1]); sys.path.insert(0,os.path.join(ROOT,'semantic'))
import calibrate_camera as cc

scratch=tempfile.mkdtemp(prefix='bagtools-calib-nav-')
try:
    path=os.path.join(scratch,'cam_20261115_120000_0.bag')
    with rosbag.Bag(path,'w') as bag:
        fixed=[]
        for parent,child,x in [('base_link','camera_link',1.),('camera_link','camera_depth_optical_frame',0.),('base_link','laser',0.)]:
            tr=TransformStamped(); tr.header.frame_id=parent; tr.child_frame_id=child
            tr.transform.translation.x=x; tr.transform.rotation.w=1.; fixed.append(tr)
        bag.write('/tf_static',TFMessage(fixed),rospy.Time.from_sec(9.9))
        for topic,fx in [(cc.RGB_INFO,200.),(cc.DEPTH_INFO,100.)]:
            info=CameraInfo(); info.width=info.height=24; info.K=[fx,0,12,0,fx,12,0,0,1]
            bag.write(topic,info,rospy.Time.from_sec(9.99))
        for i in range(6):
            t=10+i*.1; stamp=rospy.Time.from_sec(t)
            odom=Odometry(); odom.header.stamp=stamp; odom.pose.pose.orientation.w=1.
            bag.write('/odom',odom,stamp)
            scan=LaserScan(); scan.header.stamp=stamp; scan.header.frame_id='laser'
            scan.angle_min=-3.14; scan.angle_increment=6.28/360; scan.range_min=.1; scan.range_max=10.; scan.ranges=[3.]*360
            bag.write('/scan',scan,stamp)
            trs=[]
            for parent,child,x in [('map','odom',5.),('odom','base_footprint',0.)]:
                tr=TransformStamped(); tr.header.stamp=stamp; tr.header.frame_id=parent; tr.child_frame_id=child
                tr.transform.translation.x=x; tr.transform.rotation.w=1.; trs.append(tr)
            bag.write('/tf',TFMessage(trs),stamp)
            msg=Image(); msg.header.stamp=stamp; msg.header.frame_id='camera_depth_optical_frame'
            msg.width=msg.height=24; msg.encoding='16UC1'; msg.step=48; msg.data=np.full((24,24),3000,dtype='<u2').tostring()
            bag.write(cc.DEPTH,msg,stamp)
    before=hashlib.sha256(open(path,'rb').read()).hexdigest()
    with rosbag.Bag(path) as bag: pts,k,scan,frame=cc.collect(bag,4)
    assert k[0]==100 and abs(pts[:,0].max()-.24)<1e-6,(k,pts[:,0].max())
    grid=np.mgrid[-1:1:10j,-1:1:10j]
    points=np.column_stack([np.ones(100),grid[0].reshape(-1),grid[1].reshape(-1)])
    normal,_,_,count=cc.fit_plane(points,iters=20,tol=.001)
    assert abs(normal[0])>.999 and count==100
    try: cc.fit_plane(np.zeros((5,3)),iters=5)
    except ValueError: pass
    else: raise AssertionError('Degenerate plane was accepted')
    class Args: pass
    a=Args(); a.out=os.path.join(scratch,'candidate.json'); a.id='fixture_candidate'; a.bag=path
    a.height=.18; a.base_height=.08; a.fixed_yaw=0.; a.fixed_xy='.08,0'
    cc.report(np.eye(3),np.array([.08,0,.1]),np.eye(4),a)
    candidate=json.load(open(a.out)); assert candidate['estimated'] and candidate['transforms'][0]['quaternion_xyzw']==[0.,0.,0.,1.]
    out=os.path.join(scratch,'repaired')
    subprocess.check_call(['python2',os.path.join(ROOT,'semantic','prepare_navigation.py'),path,out,
        '--calibration',a.out,'--override-recorded-extrinsics','--allow-estimated-extrinsics',
        '--record-start','10.05','--record-end','10.45'])
    repair=json.load(open(os.path.join(out,'repair.json')))
    assert repair['state']=='complete' and repair['removed_map_odom']>0
    assert ['base_link','camera_link'] in repair['replaced_installation_edges']
    edges=[]
    with rosbag.Bag(repair['output']) as bag:
        assert all(c.header.get('latching')=='1' for c in bag._get_connections(topics=['/tf_static']))
        static_messages=list(bag.read_messages(topics=['/tf_static']))
        assert len(static_messages)==1
        static_edges={(t.header.frame_id,t.child_frame_id) for t in static_messages[0][1].transforms}
        assert {('base_footprint','base_link'),('base_link','camera_link'),
                ('camera_link','camera_depth_optical_frame'),('base_link','laser')}<=static_edges
        for topic,msg,_ in bag.read_messages(topics=['/tf','/tf_static']):
            edges.extend((t.header.frame_id,t.child_frame_id) for t in msg.transforms)
    assert ('map','odom') not in edges and ('base_footprint','base_link') in edges
    assert hashlib.sha256(open(path,'rb').read()).hexdigest()==before
    print(json.dumps(dict(state='passed',checks=['correct_depth_intrinsics','stationary_plane_fit',
        'degenerate_plane_rejection','candidate_config_export','filtered_navigation','explicit_tf_override','latched_static_tf','source_unchanged']),indent=2))
finally: shutil.rmtree(scratch)

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""One-shot TF existence or configured installation/publisher validation."""
from __future__ import print_function
import json
import math
import os
import sys
import threading
import time
import struct
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage

sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from calibration_schema import load_calibration, transform_error
from live_health import monotonic


def check_calibration(path):
    config=load_calibration(path)
    expected={(x['parent'],x['child']):x for x in config['transforms']}
    sources={key:set() for key in expected}
    errors=set()
    lock=threading.Lock()

    def observe(msg):
        source=(getattr(msg,'_connection_header',None) or {}).get('callerid','unknown')
        with lock:
            for item in msg.transforms:
                key=(item.header.frame_id.strip('/'),item.child_frame_id.strip('/'))
                if key not in expected:
                    continue
                sources[key].add(source)
                p,q=item.transform.translation,item.transform.rotation
                try:
                    distance,angle=transform_error([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],expected[key])
                    if distance>.001 or angle>math.radians(.1):
                        errors.add('%s -> %s differs: %.6f m, %.4f deg' % (key+(distance,math.degrees(angle))))
                except ValueError as exc:
                    errors.add(str(exc))

    subscribers=[rospy.Subscriber(topic,TFMessage,observe,queue_size=100) for topic in ['/tf','/tf_static']]
    start=monotonic()
    while not rospy.is_shutdown() and monotonic()-start<4.:
        time.sleep(.05)
        with lock:
            if monotonic()-start>=1.5 and all(sources.values()):
                break
    with lock:
        for key,publishers in sources.items():
            if len(publishers)!=1 or 'unknown' in publishers:
                errors.add('%s -> %s publishers: %s' % (key+(sorted(publishers),)))
        result=dict(calibration_id=config['id'],matched=not errors,errors=sorted(errors),
                    publishers={' -> '.join(k):sorted(v) for k,v in sources.items()})
    for sub in subscribers:
        sub.unregister()
    print(json.dumps(result,sort_keys=True))
    return 0 if result['matched'] else 1


def check_inputs(path=None):
    """Gather streams and TF in one ROS process instead of repeated CLI startups."""
    from rospy.msg import AnyMsg
    expected={'/scan':'sensor_msgs/LaserScan','/odom':'nav_msgs/Odometry',
              '/camera/rgb/image_raw':'sensor_msgs/Image','/camera/depth/image_raw':'sensor_msgs/Image',
              '/camera/rgb/camera_info':'sensor_msgs/CameraInfo','/camera/depth/camera_info':'sensor_msgs/CameraInfo'}
    observed={};lock=threading.Lock()
    buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
    def observe(msg,topic):
        try:
            length=struct.unpack_from('<I',msg._buff,12)[0]
            frame=msg._buff[16:16+length].decode('utf-8').strip('/')
            kind=msg._connection_header.get('type')
            with lock:observed[topic]=(frame,kind)
        except Exception:
            with lock:observed[topic]=('',None)
    subscribers=[rospy.Subscriber(t,AnyMsg,observe,callback_args=t,queue_size=1,buff_size=2**22) for t in expected]
    deadline=monotonic()+3.5
    while not rospy.is_shutdown() and monotonic()<deadline:time.sleep(.05)
    failures=[]
    def report(ok,message):
        print(('OK    ' if ok else 'FAIL  ')+message)
        if not ok:failures.append(message)
    with lock:streams=dict(observed)
    for topic,kind in sorted(expected.items()):
        report(topic in streams and streams[topic][1]==kind,topic+' data and type')
    for parent,child in [('odom','base_footprint'),('base_footprint','base_link'),('base_link','laser'),('base_link','camera_link')]:
        report(buffer.can_transform(parent,child,rospy.Time(0)), 'TF '+parent+' -> '+child)
    for topic in ['/camera/rgb/image_raw','/camera/depth/image_raw']:
        frame=streams.get(topic,('',None))[0]
        report(bool(frame) and buffer.can_transform('base_link',frame,rospy.Time(0)),'Optical TF for '+topic)
    if streams.get('/camera/depth/image_raw',('',None))[0]=='camera_color_optical_frame':
        print('OK    Depth registered to color optical frame')
    else:print('WARN  Unregistered depth requires explicit intrinsics and optical TF')
    if not buffer.can_transform('map','odom',rospy.Time(0)):
        print('WARN  map -> odom absent; no live SLAM map')
    if path:
        report(check_calibration(path)==0,'Installation values and one publisher match configuration')
    for subscriber in subscribers:subscriber.unregister()
    return 1 if failures else 0


if __name__=='__main__':
    rospy.init_node('bagtools_tf_check',anonymous=True)
    try:
        if len(sys.argv)==3 and sys.argv[1]=='--preflight':
            raise SystemExit(check_inputs(None if sys.argv[2]=='-' else sys.argv[2]))
        if len(sys.argv)==3 and sys.argv[1]=='--calibration':
            raise SystemExit(check_calibration(sys.argv[2]))
        buffer=tf2_ros.Buffer(); listener=tf2_ros.TransformListener(buffer)
        tr=buffer.lookup_transform(sys.argv[1],sys.argv[2],rospy.Time(0),rospy.Duration(3)).transform
        p,q=tr.translation,tr.rotation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if any(math.isnan(v) or math.isinf(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>.01:
            raise ValueError('Invalid transform values')
    except Exception as exc:
        sys.stderr.write(str(exc)+'\n'); raise SystemExit(1)

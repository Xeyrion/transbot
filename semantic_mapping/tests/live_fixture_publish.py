#!/usr/bin/env python2
"""Private-master test sensor publisher. No actuator topics."""
import os
import sys
import time
sys.path.insert(0,os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'recording'))
from live_health import monotonic
import rospy
from sensor_msgs.msg import Image,CameraInfo,LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

rospy.init_node('bagtools_test_sensors',anonymous=True)
classes={'/camera/rgb/image_raw':Image,'/camera/depth/image_raw':Image,'/scan':LaserScan,'/odom':Odometry,
         '/tf':TFMessage,'/camera/rgb/camera_info':CameraInfo,'/camera/depth/camera_info':CameraInfo}
pubs={t:rospy.Publisher(t,c,queue_size=1,latch=t.endswith('camera_info')) for t,c in classes.items()}
sequence=0
frozen_odom=None
wall_start=time.time(); monotonic_start=monotonic()
while not rospy.is_shutdown():
    drop=open(sys.argv[1]).read().strip() if os.path.exists(sys.argv[1]) else ''
    now=rospy.Time.from_sec(wall_start+monotonic()-monotonic_start)
    for topic,cls in classes.items():
        if topic==drop: continue
        if cls==CameraInfo and sequence>0: continue
        msg=cls()
        if hasattr(msg,'header'): msg.header.stamp=now; msg.header.seq=sequence; msg.header.frame_id='camera_color_optical_frame'
        if topic=='/odom' and drop=='odom_frozen':
            if frozen_odom is None:frozen_odom=now
            msg.header.stamp=frozen_odom
        if cls==Image:
            msg.width=12; msg.height=8; msg.encoding='rgb8' if 'rgb' in topic else '16UC1'
            msg.step=36 if 'rgb' in topic else 24
            msg.data=b'\0'*(msg.step*8) if 'rgb' in topic else b'\xd0\x07'*(12*8)
        elif cls==CameraInfo:
            msg.width=12; msg.height=8; msg.K=[100,0,6,0,100,4,0,0,1]
        elif cls==Odometry:
            msg.header.frame_id='odom'; msg.child_frame_id='base_footprint'; msg.pose.pose.orientation.w=1.
        elif cls==LaserScan:
            msg.header.frame_id='laser'; msg.range_min=.1; msg.range_max=10.; msg.angle_min=-3.14; msg.angle_increment=6.28/360; msg.ranges=[3.]*360
        elif cls==TFMessage:
            tr=TransformStamped(); tr.header.stamp=now; tr.header.frame_id='odom'; tr.child_frame_id='base_footprint'; tr.transform.rotation.w=1.
            if drop=='tf_clock_back':tr.header.stamp=now-rospy.Duration(.646171)
            msg.transforms=[tr]
            for parent,child in [('base_footprint','base_link'),('base_link','camera_link'),
                                 ('camera_link','camera_color_optical_frame'),('base_link','laser')]:
                tr=TransformStamped(); tr.header.stamp=now; tr.header.frame_id=parent; tr.child_frame_id=child; tr.transform.rotation.w=1.
                msg.transforms.append(tr)
        pubs[topic].publish(msg)
    sequence+=1; time.sleep(.05)

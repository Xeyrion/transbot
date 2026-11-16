#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Offline ROS integration fixtures; run explicitly with python2 and project root."""
from __future__ import print_function
import json
import os
import shutil
import subprocess
import sys
import tempfile
import numpy as np
import rosbag
import rospy
from PIL import Image as PillowImage
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

ROOT=os.path.abspath(sys.argv[1]); sys.path.insert(0,os.path.join(ROOT,'semantic')); sys.path.insert(0,ROOT)
import extract_frames as ef
import extract_map


def tr(parent,child,x=0.,stamp=0.):
    t=TransformStamped(); t.header.frame_id=parent; t.child_frame_id=child
    t.header.stamp=rospy.Time.from_sec(stamp); t.transform.translation.x=x; t.transform.rotation.w=1.
    return t


scratch=tempfile.mkdtemp(prefix='bagtools-geometry-')
try:
    path=os.path.join(scratch,'cam_fixture_0.bag')
    with rosbag.Bag(path,'w') as bag:
        fixed=[tr('base_footprint','base_link'),tr('base_link','camera_link',1.),
               tr('camera_link','color_optical'),tr('camera_link','depth_optical',.1)]
        bag.write('/tf_static',TFMessage(fixed),rospy.Time.from_sec(9.9))
        for topic,frame in [(ef.RGB_INFO,'color_optical'),(ef.DEPTH_INFO,'depth_optical')]:
            info=CameraInfo(); info.width=12; info.height=8; info.header.frame_id=frame
            info.K=[100.,0.,6.,0.,100.,4.,0.,0.,1.]
            bag.write(topic,info,rospy.Time.from_sec(9.99))
        for n,stamp in enumerate([10.,10.1,10.2,10.3]):
            bag.write('/tf',TFMessage([tr('odom','base_footprint',stamp-10.,stamp)]),rospy.Time.from_sec(stamp))
        for n,stamp in enumerate([10.05,10.15,10.25]):
            for topic,frame,s,encoding,data,step in [
                (ef.RGB,'color_optical',stamp,'rgb8',np.zeros((8,12,3),np.uint8).tostring(),36),
                (ef.DEPTH,'depth_optical',stamp+.005,'16UC1',np.full((8,12),2000,dtype='<u2').tostring(),24)]:
                msg=Image(); msg.header.seq=n; msg.header.frame_id=frame; msg.header.stamp=rospy.Time.from_sec(s)
                msg.width=12; msg.height=8; msg.encoding=encoding; msg.step=step; msg.data=data
                bag.write(topic,msg,rospy.Time.from_sec(s+.002))
    out=os.path.join(scratch,'frames')
    subprocess.check_call(['python2',os.path.join(ROOT,'semantic','extract_frames.py'),path,out,
                           '--max','3','--step','.09','--root','odom'])
    sampling=json.load(open(os.path.join(out,'sampling.json')))
    assert sampling['frames']==3
    meta=json.load(open(os.path.join(out,'0000','meta.json')))
    assert abs(meta['t_world_optical'][0][3]-1.05)<1e-6,meta
    assert abs(meta['pair_delta_ms']-5.)<1e-5,meta
    assert meta['frame']=='color_optical' and meta['source_depth_frame']=='depth_optical'
    d=np.load(os.path.join(out,'0000','depth.npy'))
    assert d.shape==(8,12) and d[4,6]==2. and d[4,0]==0.
    # Intrinsics published once before a selected segment remain usable.
    segment=os.path.join(scratch,'segment')
    subprocess.check_call(['python2',os.path.join(ROOT,'semantic','extract_frames.py'),path,segment,
        '--max','1','--record-start','10.14','--record-end','10.29','--root','odom'])
    # Big-endian depth values and correct target resolution.
    msg=Image(); msg.width=2; msg.height=1; msg.step=4; msg.encoding='16UC1'; msg.is_bigendian=1
    msg.data=np.array([1000,2000],dtype='>u2').tostring()
    assert np.allclose(ef.depth_array(msg),[[1.,2.]])
    k=[1.,0,0,0,1.,0,0,0,1.]
    identity=[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[0,0,0,1.]]
    target=ef.reproject(np.ones((1,1)),k,k,identity,target_shape=(3,4))
    assert target.shape==(3,4) and target[0,0]==1.
    class Box: pass
    grid=Box(); grid.info=Box(); grid.info.width=2; grid.info.height=2; grid.data=[0,100,-1,50]
    pgm=os.path.join(scratch,'map.pgm'); extract_map.write_pgm(pgm,grid)
    assert np.asarray(PillowImage.open(pgm)).tolist()==[[205,127],[254,0]]
    # Existing output must be refused, including when fewer frames are requested.
    proc=subprocess.Popen(['python2',os.path.join(ROOT,'semantic','extract_frames.py'),path,out,'--max','1'],
                          stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    stdout,stderr=proc.communicate(); assert proc.returncode!=0
    image_out=os.path.join(scratch,'label_images')
    subprocess.check_call(['python2',os.path.join(ROOT,'semantic','extract_images.py'),path,image_out,
                           '--fps','30','--max','3','--diff','0'])
    images=json.load(open(os.path.join(image_out,'images.json')))
    assert images['state']=='complete' and len(set(x['name'] for x in images['images']))==3
    import extract_images
    bgr=Image(); bgr.width=bgr.height=1; bgr.encoding='bgr8'; bgr.step=3; bgr.data=bytes(bytearray([1,2,3]))
    assert extract_images.to_array(bgr).tolist()==[[[3,2,1]]]
    # Independent image export can sample record time despite a sensor clock reset.
    reset_path=os.path.join(scratch,'cam_20260101_000000_0.bag')
    pixels=np.arange(18,dtype=np.uint8).reshape(2,3,3)
    with rosbag.Bag(reset_path,'w') as bag:
        for n,stamp in enumerate([30.,31.,29.]):
            msg=Image(); msg.width=3; msg.height=2; msg.step=12; msg.encoding='bgr8'
            msg.header.seq=n; msg.header.stamp=rospy.Time.from_sec(stamp)
            # Nonzero row padding must not leak into the lossless output.
            padded=np.full((2,12),255,dtype=np.uint8)
            padded[:,:9]=pixels[:,:,::-1].reshape(2,9)
            msg.data=padded.tostring()
            bag.write(extract_images.RGB,msg,rospy.Time.from_sec(40.+n))
    with rosbag.Bag(os.path.join(scratch,'cam_20260101_000000_1.bag'),'w') as bag:
        msg.header.stamp=rospy.Time.from_sec(28.); msg.header.seq=3
        bag.write(extract_images.RGB,msg,rospy.Time.from_sec(39.))
    png_out=os.path.join(scratch,'png')
    subprocess.check_call(['python2',os.path.join(ROOT,'semantic','extract_images.py'),reset_path,
        png_out,'--format','png','--time-source','record','--diff','0','--fps','1'])
    png_manifest=json.load(open(os.path.join(png_out,'images.json')))
    assert png_manifest['state']=='complete' and len(png_manifest['images'])==4
    assert len(png_manifest['header_regressions'])==2 and len(png_manifest['record_regressions'])==1
    for n,item in enumerate(png_manifest['images']):
        assert item['record_stamp']==[40.,41.,42.,39.][n] and item['width']==3 and item['height']==2
        assert item['record_epoch']==(1 if n==3 else 0)
        assert item['name'].endswith('.png')
        assert np.array_equal(np.asarray(PillowImage.open(os.path.join(png_out,item['name']))),pixels)
    failed_out=os.path.join(scratch,'header_reset')
    proc=subprocess.Popen(['python2',os.path.join(ROOT,'semantic','extract_images.py'),reset_path,
        failed_out,'--diff','0'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    proc.communicate(); assert proc.returncode!=0
    failed=json.load(open(os.path.join(failed_out,'images.json')))
    assert failed['state']=='failed' and 'regressed' in failed['error']
    print(json.dumps(dict(state='passed',checks=['moving_rgb_depth_geometry','bounded_header_pairing',
        'latched_intrinsics_in_segment','big_endian_depth','target_dimensions','map_vertical_orientation',
        'output_reuse_refused','subsecond_image_names','bgr_channel_conversion',
        'record_time_clock_reset_export','lossless_png_with_stride','failed_export_manifest']),indent=2))
finally:
    shutil.rmtree(scratch)

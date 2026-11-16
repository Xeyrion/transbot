#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Local ROS observer. Raw message callbacks avoid deserializing full images."""
from __future__ import print_function
import argparse
import ctypes
import json
import os
import struct
import sys
import threading
import time
from stream_health import StreamHealth, LATCHED, ODOM_TF


def monotonic():
    if hasattr(time,'monotonic'): return time.monotonic()
    class Timespec(ctypes.Structure):
        _fields_=[('sec',ctypes.c_long),('nsec',ctypes.c_long)]
    value=Timespec()
    if ctypes.CDLL(None).clock_gettime(1,ctypes.byref(value)) != 0: raise RuntimeError('clock_gettime failed')
    return value.sec+value.nsec/1e9


def rates_main():
    """Read-only local counters for the GUI, independent of bag recording."""
    import rospy
    from rospy.msg import AnyMsg
    rospy.init_node('bagtools_source_status',anonymous=True,disable_signals=True)
    topics=['/camera/rgb/image_raw','/camera/depth/image_raw','/scan']
    streams={};lock=threading.Lock();previous={};sample_time=monotonic()
    def observe(msg,topic):
        with lock:
            old=streams.get(topic,{})
            streams[topic]=dict(count=old.get('count',0)+1,last_monotonic=monotonic())
    subscribers=[rospy.Subscriber(t,AnyMsg,observe,callback_args=t,queue_size=1,buff_size=2**22) for t in topics]
    while not rospy.is_shutdown():
        with lock:
            snapshot=dict(updated_monotonic=monotonic(),streams={t:dict(v) for t,v in streams.items()})
        now=snapshot['updated_monotonic'];dt=now-sample_time
        snapshot['rates']={t:round(max(0,snapshot['streams'].get(t,{}).get('count',0)-previous.get(t,0))/dt,1) if dt>=.1 else -1. for t in topics}
        previous={t:v['count'] for t,v in snapshot['streams'].items()};sample_time=now
        sys.stdout.write(json.dumps(snapshot)+'\n');sys.stdout.flush();time.sleep(1.)
    return 0


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('profile',choices=['full','nav']); ap.add_argument('output')
    ap.add_argument('--timeout',type=float,default=3.)
    ap.add_argument('--rates-only',action='store_true'); args=ap.parse_args()
    if args.rates_only:
        return rates_main()
    health=StreamHealth(args.profile,monotonic(),timeout=args.timeout); lock=threading.Lock()
    def save(snapshot):
        snapshot['observer_pid']=os.getpid()
        with open(args.output+'.tmp','w') as f: json.dump(snapshot,f)
        os.rename(args.output+'.tmp',args.output)
    save(health.snapshot(monotonic()))
    try:
        import rospy
        from rospy.msg import AnyMsg
        from tf2_msgs.msg import TFMessage
        rospy.init_node('bagtools_stream_health',anonymous=True,disable_signals=True)
        def callback(msg,topic):
            now=monotonic()
            stamp=None
            if topic == '/tf':
                try:
                    transforms=TFMessage().deserialize(msg._buff).transforms
                    with lock:
                        for tr in transforms:
                            if (tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/'))==('odom','base_footprint'):
                                health.observe(ODOM_TF,now,tr.header.stamp.to_sec())
                except Exception:
                    with lock: health.fault='invalid_tf_message'
            elif topic not in LATCHED:
                try:
                    _,sec,nsec=struct.unpack_from('<III',msg._buff)
                    stamp=sec+nsec/1e9
                except Exception:
                    with lock: health.fault='invalid_header:'+topic
            with lock: health.observe(topic,now,stamp)
        subscribers=[rospy.Subscriber(t,AnyMsg,callback,callback_args=t,queue_size=1,buff_size=2**22)
                     for t in health.subscriptions]
        while not rospy.is_shutdown():
            with lock: snapshot=health.snapshot(monotonic())
            save(snapshot); time.sleep(.2)
    except Exception as exc:
        save(dict(state='failed',faults=['observer_error:'+str(exc)],updated_monotonic=monotonic()))
        return 1
    return 0


if __name__=='__main__': raise SystemExit(main())

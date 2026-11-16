#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Local ROS observer. Raw message callbacks avoid deserializing full images."""
from __future__ import print_function
import argparse
import ctypes
import json
import os
import struct
import threading
import time
from stream_health import StreamHealth, LATCHED


def monotonic():
    if hasattr(time,'monotonic'): return time.monotonic()
    class Timespec(ctypes.Structure):
        _fields_=[('sec',ctypes.c_long),('nsec',ctypes.c_long)]
    value=Timespec()
    if ctypes.CDLL(None).clock_gettime(1,ctypes.byref(value)) != 0: raise RuntimeError('clock_gettime failed')
    return value.sec+value.nsec/1e9


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('profile',choices=['full','nav']); ap.add_argument('output')
    ap.add_argument('--timeout',type=float,default=3.); args=ap.parse_args()
    health=StreamHealth(args.profile,monotonic(),timeout=args.timeout); lock=threading.Lock()
    def save(snapshot):
        snapshot['observer_pid']=os.getpid()
        with open(args.output+'.tmp','w') as f: json.dump(snapshot,f)
        os.rename(args.output+'.tmp',args.output)
    save(health.snapshot(monotonic()))
    try:
        import rospy
        from rospy.msg import AnyMsg
        rospy.init_node('bagtools_stream_health',anonymous=True,disable_signals=True)
        def callback(msg,topic):
            stamp=None
            if topic != '/tf' and topic not in LATCHED:
                try:
                    _,sec,nsec=struct.unpack_from('<III',msg._buff)
                    stamp=sec+nsec/1e9
                except Exception:
                    with lock: health.fault='invalid_header:'+topic
            with lock: health.observe(topic,monotonic(),stamp)
        subscribers=[rospy.Subscriber(t,AnyMsg,callback,callback_args=t,queue_size=1,buff_size=2**22)
                     for t in health.required]
        while not rospy.is_shutdown():
            with lock: snapshot=health.snapshot(monotonic())
            save(snapshot); time.sleep(.2)
    except Exception as exc:
        save(dict(state='failed',faults=['observer_error:'+str(exc)],updated_monotonic=monotonic()))
        return 1
    return 0


if __name__=='__main__': raise SystemExit(main())

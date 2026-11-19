#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import json
import os
import sys
sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ros_runtime import ros_python, ros_python_major
if __name__ == '__main__' and sys.version_info[0] != ros_python_major():
    os.execvp(ros_python(), [ros_python(), os.path.abspath(__file__)]+sys.argv[1:])
import rosbag
from viewer_core import session_files
from time_segments import review

ap=argparse.ArgumentParser(description='Read-only temporal review; output is a NEW JSON, not a modified bag')
ap.add_argument('bag'); ap.add_argument('out'); ap.add_argument('--max-gap',type=float,default=1.)
ap.add_argument('--margin',type=float,default=.5); args=ap.parse_args()
if os.path.exists(args.out): ap.error('Output exists')
files=session_files(args.bag)
if not files: ap.error('No sources')
names={'/camera/rgb/image_raw':'rgb','/camera/depth/image_raw':'depth','/scan':'scan','/odom':'odom'}
streams={name:[] for name in names.values()}
sources=[dict(path=p,size=os.path.getsize(p),mtime=os.path.getmtime(p)) for p in files]
for path in files:
    with rosbag.Bag(path) as bag:
        for topic,msg,rec in bag.read_messages(topics=list(names)):
            streams[names[topic]].append((rec.to_sec(),msg.header.stamp.to_sec()))
for values in streams.values(): values.sort(key=lambda v:v[0])
result=review(streams,args.max_gap,args.margin); result.update(schema_version=1,sources=sources)
if any(os.path.getsize(s['path'])!=s['size'] or os.path.getmtime(s['path'])!=s['mtime'] for s in sources):
    raise ValueError('Sources changed during review')
fd=os.open(args.out,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o644)
with os.fdopen(fd,'w') as f: json.dump(result,f,indent=2)
print('Candidate continuous windows:',len(result['segments']))

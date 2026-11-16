#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Inspect navigation clocks and select one continuous interval, without changing bags."""
from __future__ import print_function
import json
import os
import sys
import rosbag
sys.path.insert(0,os.path.join(os.path.dirname(os.path.abspath(__file__)),'semantic'))
from time_segments import review
from viewer_core import session_files


def inspect(path):
    sources=session_files(path)
    if not sources:raise ValueError('No source bags')
    streams={'scan':[],'odom':[],'tf_odom':[]};edges=set();frames=set();maps=[]
    for source in sources:
        with rosbag.Bag(source) as bag:
            for topic,msg,rec,connection in bag.read_messages(
                    topics=['/scan','/odom','/tf','/tf_static','/map'],return_connection_header=True):
                if topic in ('/scan','/odom'):
                    streams[topic[1:]].append((rec.to_sec(),msg.header.stamp.to_sec()))
                    if topic=='/scan':frames.add(msg.header.frame_id.strip('/'))
                elif topic=='/map':
                    maps.append(dict(caller=connection.get('callerid'),header=msg.header.stamp.to_sec(),record=rec.to_sec()))
                else:
                    for tr in msg.transforms:
                        key=(tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/'))
                        if key==('map','odom'):continue
                        edges.add(key)
                        if topic=='/tf' and key==('odom','base_footprint'):
                            streams['tf_odom'].append((rec.to_sec(),tr.header.stamp.to_sec()))
    if len(frames)!=1:raise ValueError('Missing or changing laser frame: '+repr(sorted(frames)))
    graph={}
    for parent,child in sorted(edges):
        if child in graph and graph[child]!=parent:raise ValueError('Multiple TF parents: '+child)
        graph[child]=parent
    child=next(iter(frames));seen=set()
    while child!='base_footprint':
        if child in seen or child not in graph:raise ValueError('Missing navigation TF: base_footprint -> '+next(iter(frames)))
        seen.add(child);child=graph[child]
    for values in streams.values():values.sort(key=lambda v:v[0])
    checked=review(streams,margin=.5)
    if not checked['segments']:raise ValueError('No continuous scan/odom/TF interval')
    best=max(checked['segments'],key=lambda v:v['record_end']-v['record_start'])
    partial=bool(checked['issues'])
    if partial and best['record_end']-best['record_start']<5:
        raise ValueError('Continuous navigation intervals are too short (<5 s)')
    return dict(schema_version=1,sources=[dict(path=p,size=os.path.getsize(p),mtime=os.path.getmtime(p)) for p in sources],
        record_bounds=[best['record_start'],best['record_end']] if partial else [None,None],
        partial=partial,selected=best,segments=checked['segments'],issues=checked['issues'],recorded_maps=maps,
        counts={k:len(v) for k,v in streams.items()},laser_frame=next(iter(frames)),
        input_duration_s=min(v[-1][0] for v in streams.values())-max(v[0][0] for v in streams.values()))


if __name__=='__main__':
    try:print(json.dumps(inspect(sys.argv[1])))
    except Exception as exc:
        print(str(exc),file=sys.stderr);sys.exit(1)

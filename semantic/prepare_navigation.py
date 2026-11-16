#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Read-only source repair: new navigation-only bag, explicit TF provenance."""
from __future__ import print_function
import argparse
import json
import os
import sys
import rosbag
import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from viewer_core import session_files
from calibration_schema import load_calibration
import pose


def write_static(bag,msg,stamp):
    bag.write('/tf_static',msg,stamp,connection_header=dict(topic='/tf_static',type=msg._type,
        md5sum=msg._md5sum,message_definition=msg._full_text,latching='1'))


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('outdir')
    ap.add_argument('--calibration'); ap.add_argument('--allow-estimated-extrinsics',action='store_true')
    ap.add_argument('--override-recorded-extrinsics',action='store_true')
    ap.add_argument('--record-start',type=float); ap.add_argument('--record-end',type=float)
    ap.add_argument('--allow-gaps',action='store_true'); args=ap.parse_args()
    sources=session_files(args.bag)
    if not sources: ap.error('No source bags')
    if os.path.exists(args.outdir): ap.error('Repair output exists')
    if args.record_start is not None and args.record_end is not None and args.record_start>=args.record_end:
        ap.error('Invalid record time interval')
    config=None
    if args.calibration:
        config=load_calibration(args.calibration)
    transforms={}; installed={}; statics={}; removed=0; counts={}; previous={}; gaps=[]
    # Inspect all static installation edges before choosing replacements.
    for source in sources:
        with rosbag.Bag(source) as bag:
            for topic,msg,_ in bag.read_messages(topics=['/tf','/tf_static']):
                for tr in msg.transforms:
                    key=(tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/'))
                    installed.setdefault(key,tr)
                    if topic=='/tf_static':
                        p,q=tr.transform.translation,tr.transform.rotation
                        matrix=pose.transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
                        if key in statics:
                            old=statics[key].transform; p0,q0=old.translation,old.rotation
                            before=pose.transform([p0.x,p0.y,p0.z],[q0.x,q0.y,q0.z,q0.w])
                            if max(abs(before[i][j]-matrix[i][j]) for i in range(4) for j in range(4))>1e-5:
                                raise ValueError('Conflicting static TF: '+str(key))
                        statics[key]=tr
    for item in (config['transforms'] if config else []):
        key=(item['parent'].strip('/'),item['child'].strip('/'))
        matrix=pose.transform(item['xyz'],item['quaternion_xyzw'])
        if key in installed:
            tr=installed[key].transform; p,q=tr.translation,tr.rotation
            old=pose.transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
            if max(abs(old[i][j]-matrix[i][j]) for i in range(4) for j in range(4))<1e-5: continue
            if not args.override_recorded_extrinsics:
                raise ValueError('Recorded installation exists; explicit --override-recorded-extrinsics is required')
        transforms[key]=(item['xyz'],item['quaternion_xyzw'])
    estimated=[]
    if args.allow_estimated_extrinsics:
        for key,(xyz,yaw) in pose.FILL_INS.items():
            if key not in installed and key not in transforms:
                transforms[key]=(xyz,pose.yaw_quat(yaw)); estimated.append(list(key))
    # Laser SLAM does not depend on a camera installation transform.
    required=[('base_footprint','base_link')]
    missing=[list(k) for k in required if k not in installed and k not in transforms]
    if missing: raise ValueError('Missing installation TF; supply calibration or explicit estimates: '+str(missing))
    # rosbag play latches only the last message on one /tf_static connection.
    # Publish the complete static tree together, including explicit replacements.
    fixed=[statics[k] for k in sorted(statics) if k not in transforms and k!=('map','odom')]
    for (parent,child),(xyz,q) in sorted(transforms.items()):
        tr=TransformStamped(); tr.header.frame_id=parent; tr.child_frame_id=child
        tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=xyz
        tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=q
        fixed.append(tr)
    os.makedirs(args.outdir)
    path=os.path.join(args.outdir,os.path.basename(sources[0])); active=path+'.active'
    result=dict(schema_version=1,state='running',sources=sources,record_bounds=[args.record_start,args.record_end],
        estimated_extrinsics=estimated,calibration=config,replaced_installation_edges=[list(k) for k in transforms if k in installed],
        output=path,removed_map_odom=0)
    try:
        with rosbag.Bag(active,'w',compression='lz4') as out:
            injected=False
            for source in sources:
                with rosbag.Bag(source) as bag:
                    for topic,msg,rec in bag.read_messages(topics=['/scan','/odom','/tf']):
                        stamp=rec.to_sec()
                        if args.record_start is not None and stamp<args.record_start: continue
                        if args.record_end is not None and stamp>args.record_end: continue
                        if not injected:
                            if fixed: write_static(out,TFMessage(fixed),rec)
                            injected=True
                        if topic=='/tf':
                            keep=[]
                            for tr in msg.transforms:
                                key=(tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/'))
                                if key==('map','odom'): removed+=1; continue
                                if key not in transforms: keep.append(tr)
                            if not keep: continue
                            msg=TFMessage(keep)
                        else:
                            hdr=msg.header.stamp.to_sec()
                            if hdr<=0: raise ValueError('Invalid navigation header timestamp')
                            if topic in previous:
                                delta=hdr-previous[topic]
                                if delta < -1e-6: raise ValueError('Clock regression; select a clean record-time segment')
                                if delta>1.:
                                    gaps.append(dict(topic=topic,header_gap=delta,record=stamp))
                                    if not args.allow_gaps: raise ValueError('Navigation gap >1s; select a continuous segment or explicitly --allow-gaps')
                            previous[topic]=hdr
                        out.write(topic,msg,rec); counts[topic]=counts.get(topic,0)+1
        if not counts.get('/scan') or not counts.get('/odom'): raise ValueError('Missing navigation observations')
        os.rename(active,path); result.update(state='complete',counts=counts,removed_map_odom=removed,gaps=gaps)
    except Exception as exc:
        result.update(state='failed',error=str(exc)); raise
    finally:
        with open(os.path.join(args.outdir,'repair.json'),'w') as f: json.dump(result,f,indent=2)
    print(path)


if __name__=='__main__': main()

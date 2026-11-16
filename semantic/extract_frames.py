#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Read-only bag extraction with bounded RGB-D pairing and explicit geometry."""
from __future__ import print_function
import argparse
import bisect
from collections import Counter
import glob
import json
import os
import sys

if sys.version_info[0] != 2:
    sys.exit('Use python2 extract_frames.py (ROS Melodic rosbag)')

import numpy as np
import rosbag
import rospy
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import pose
from sampling import HeaderIndex
from camera_geometry import coefficients,distort,undistort
from viewer_core import session_files, association
from calibration_schema import load_calibration,ALLOWED_EDGES

RGB = '/camera/rgb/image_raw'
DEPTH = '/camera/depth/image_raw'
DEPTH_INFO = '/camera/depth/camera_info'
RGB_INFO = '/camera/rgb/camera_info'


def in_bounds(stamp, bounds):
    return ((bounds[0] is None or stamp >= bounds[0]) and
            (bounds[1] is None or stamp <= bounds[1]))


def absorb(tf, bag, bounds, skip_map_odom=False,skip_edges=()):
    for topic, msg, rec in bag.read_messages(topics=['/tf_static', '/tf']):
        if topic != '/tf_static' and not in_bounds(rec.to_sec(), bounds):
            continue
        for x in msg.transforms:
            if (x.header.frame_id.strip('/'),x.child_frame_id.strip('/')) in skip_edges: continue
            if skip_map_odom and (x.header.frame_id.strip('/'),x.child_frame_id.strip('/'))==('map','odom'):
                continue
            tr = x.transform
            tf.add(x.header.frame_id, x.child_frame_id, x.header.stamp.to_sec(),
                   pose.transform((tr.translation.x, tr.translation.y, tr.translation.z),
                                  (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)),
                   static=topic == '/tf_static')


def load_tf(files, log, bounds=(None, None), allow_estimated=False, calibration=None, map_manifest=None, override=False):
    tf = pose.Tf()
    linked = association(files[0], manifest=map_manifest)
    trajectory = linked.get('trajectory') if linked else None
    for path in files + ([trajectory] if trajectory else []):
        with rosbag.Bag(path) as bag:
            absorb(tf, bag, bounds, skip_map_odom=bool(path in files and (not linked or linked.get('replace_map_odom'))),
                   skip_edges=[tuple(k) for k in linked.get('replace_installation_edges',[])] if linked and path in files else ())
    tf.calibration_id = None
    tf.calibration_estimated = False
    replaced = []
    if calibration:
        data=load_calibration(calibration)
        tf.calibration_id = data['id']
        tf.calibration_estimated = bool(data.get('estimated',False))
        for item in data['transforms']:
            key = (item['parent'].strip('/'), item['child'].strip('/'))
            matrix = pose.transform(item['xyz'], item['quaternion_xyzw'])
            if key in tf.edges:
                if any(max(abs(m[i][j]-matrix[i][j]) for i in range(4) for j in range(4)) > 1e-4
                       for _, m in tf.edges[key]):
                    if not override or key not in ALLOWED_EDGES:
                        raise ValueError('Calibration conflicts with recorded TF: ' + str(key))
                    replaced.append(dict(parent=key[0],child=key[1],old=tf.edges[key][0][1]))
                    del tf.edges[key]; tf.static.pop(key,None)
                    tf.add(key[0],key[1],0.,matrix,static=True)
            else:
                tf.add(key[0], key[1], 0., matrix, static=True)
    tf.fill_known(log, allow=allow_estimated)
    missing = [key for key in pose.FILL_INS if key not in tf.edges]
    if missing:
        raise ValueError('Missing installation TF %s; supply --calibration, or explicitly use --allow-estimated-extrinsics for exploratory results' % missing)
    repair=linked.get('repair',{}) if linked else {}
    repair_cal=repair.get('calibration') or {}
    tf.geometry = dict(map=linked, calibration_id=tf.calibration_id or repair_cal.get('id'),
                       calibration_estimated=tf.calibration_estimated or repair_cal.get('estimated',False),replaced_recorded_extrinsics=replaced,
                       estimated_extrinsics=[list(k) for k in tf.estimated]+repair.get('estimated_extrinsics',[]))
    return tf


def depth_array(msg):
    if msg.encoding not in ('16UC1', 'mono16', '32FC1'):
        raise ValueError('Unsupported depth encoding: ' + msg.encoding)
    size = 4 if msg.encoding == '32FC1' else 2
    if msg.step < msg.width * size or len(msg.data) != msg.step * msg.height:
        raise ValueError('Invalid depth image dimensions/stride')
    dtype = ('>' if msg.is_bigendian else '<') + ('f4' if size == 4 else 'u2')
    d = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // size)[:, :msg.width]
    return d.astype(np.float32) / (1. if size == 4 else 1000.)


def reproject(d, k_depth, k_color, m, max_range=8., target_shape=None, d_depth=None, d_color=None):
    h, w = d.shape
    th, tw = target_shape or (h, w)
    v, u = np.mgrid[0:h, 0:w]
    ok = np.isfinite(d) & (d > .2) & (d < max_range)
    z = d[ok]
    xx,yy=undistort((u[ok]-k_depth[2])/k_depth[0],(v[ok]-k_depth[5])/k_depth[4],d_depth)
    points = np.stack([xx*z,yy*z,z],axis=1)
    m = np.asarray(m)
    q = points.dot(m[:3, :3].T) + m[:3, 3]
    q = q[np.isfinite(q).all(axis=1) & (q[:, 2] > .2)]
    xx,yy=distort(q[:,0]/q[:,2],q[:,1]/q[:,2],d_color)
    finite=np.isfinite(xx)&np.isfinite(yy); xx,yy,q=xx[finite],yy[finite],q[finite]
    uu = np.rint(xx*k_color[0]+k_color[2]).astype(np.int64)
    vv = np.rint(yy*k_color[4]+k_color[5]).astype(np.int64)
    inside = (uu >= 0) & (uu < tw) & (vv >= 0) & (vv < th)
    out = np.full((th*tw,), np.inf, np.float32)
    np.minimum.at(out, vv[inside]*tw+uu[inside], q[inside, 2])
    out[~np.isfinite(out)] = 0
    return out.reshape(th, tw)


def read_entry(bag, topic, entry):
    for _, msg, rec in bag.read_messages(topics=[topic],
            start_time=rospy.Time.from_sec(entry['record'] - 1e-6),
            end_time=rospy.Time.from_sec(entry['record'] + 1e-6)):
        if abs(msg.header.stamp.to_sec()-entry['header']) < 1e-7 and msg.header.seq == entry['seq']:
            return msg
    raise ValueError('Indexed image changed or could not be read')


def camera_info(entries, record, width, height):
    choices = [v for v in entries if v[1].width == width and v[1].height == height]
    if not choices:
        return None
    i = bisect.bisect_right([v[0] for v in choices], record)
    msg = choices[max(0, i-1)][1]
    if not np.isfinite(msg.K).all() or msg.K[0] <= 0 or msg.K[4] <= 0:
        return None
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('outdir')
    ap.add_argument('--step', type=float, default=1.)
    ap.add_argument('--max', type=int, default=120)
    ap.add_argument('--max-pair-ms', type=float, default=33.)
    ap.add_argument('--root', choices=['map', 'odom'], default=None)
    ap.add_argument('--record-start', type=float)
    ap.add_argument('--record-end', type=float)
    ap.add_argument('--calibration')
    ap.add_argument('--map-association')
    ap.add_argument('--override-recorded-extrinsics',action='store_true')
    ap.add_argument('--allow-estimated-extrinsics', action='store_true')
    args = ap.parse_args()
    if args.step <= 0 or args.max <= 0 or args.max_pair_ms <= 0:
        ap.error('step, max, and max-pair-ms must be positive')
    if args.record_start is not None and args.record_end is not None and args.record_start >= args.record_end:
        ap.error('record-start must be before record-end')
    if os.path.isdir(args.outdir) and any(glob.glob(os.path.join(args.outdir, pattern))
            for pattern in ['[0-9][0-9][0-9][0-9]', 'sampling.json', 'observations.json', 'objects.json']):
        ap.error('Output contains previous results; use a new run directory')
    if not os.path.isdir(args.outdir): os.makedirs(args.outdir)
    files = session_files(args.bag)
    linked=association(files[0],manifest=args.map_association) if files else None
    bounds = (args.record_start, args.record_end)
    if linked and linked.get('record_bounds'):
        lo,hi=linked['record_bounds']
        bounds=(args.record_start if args.record_start is not None else lo,
                args.record_end if args.record_end is not None else hi)
        if (lo is not None and bounds[0]<lo) or (hi is not None and bounds[1]>hi):
            ap.error('Requested segment exceeds mapped trajectory interval')
    bags = {}
    rgb_index, depth_index = HeaderIndex(), HeaderIndex()
    infos = {RGB_INFO: [], DEPTH_INFO: []}
    rejected = Counter()
    report = dict(schema_version=2, state='indexing', files=files, frames=0,
                  max_pair_ms=args.max_pair_ms, record_bounds=bounds)
    try:
        for path in files:
            bag = rosbag.Bag(path); bags[path] = bag
            print('Indexing image acquisition timestamps:', os.path.basename(path))
            for topic, msg, rec in bag.read_messages(topics=[RGB, DEPTH, RGB_INFO, DEPTH_INFO]):
                if topic in infos:
                    infos[topic].append((rec.to_sec(), msg))
                else:
                    if not in_bounds(rec.to_sec(), bounds): continue
                    index = rgb_index if topic == RGB else depth_index
                    index.add(msg.header.stamp.to_sec(), rec.to_sec(), path, msg.header.seq)
        for entries in infos.values(): entries.sort(key=lambda v: v[0])
        tf = load_tf(files, print, bounds, args.allow_estimated_extrinsics, args.calibration,
                     args.map_association,args.override_recorded_extrinsics)
        world = args.root or ('map' if ('map', 'odom') in tf.edges else 'odom')
        report.update(world_frame=world, geometry=tf.geometry, indexed_rgb=len(rgb_index.entries),
                      indexed_depth=len(depth_index.entries))
        for entry in rgb_index.sampled(args.step):
            if report['frames'] >= args.max: break
            pair = depth_index.nearest(entry['header'], args.max_pair_ms/1000.)
            if pair is None:
                rejected['pair_outside_limit'] += 1; continue
            rgb = read_entry(bags[entry['path']], RGB, entry)
            depth = read_entry(bags[pair['path']], DEPTH, pair)
            rinfo = camera_info(infos[RGB_INFO], entry['record'], rgb.width, rgb.height)
            dinfo = camera_info(infos[DEPTH_INFO], pair['record'], depth.width, depth.height)
            if rinfo is None or dinfo is None:
                rejected['missing_intrinsics'] += 1; continue
            coefficients(list(rinfo.D),rinfo.distortion_model or 'plumb_bob')
            coefficients(list(dinfo.D),dinfo.distortion_model or 'plumb_bob')
            frame = depth.header.frame_id.strip('/')
            color_frame = rgb.header.frame_id.strip('/')
            matrix = tf.chain_to(color_frame, entry['header'], root=world)
            if matrix is None:
                rejected['missing_or_stale_world_tf'] += 1; continue
            d = depth_array(depth); aligned = frame == color_frame
            world_depth = tf.chain_to(frame, pair['header'], root=world)
            if world_depth is None:
                rejected['missing_or_stale_depth_tf'] += 1; continue
            color_from_depth = pose.matmul(pose.inverse(matrix), world_depth)
            if aligned:
                if d.shape != (rgb.height, rgb.width):
                    rejected['aligned_size_mismatch'] += 1; continue
                identity = np.eye(4)
                if not np.allclose(color_from_depth, identity, atol=1e-8):
                    d = reproject(d, list(rinfo.K), list(rinfo.K), color_from_depth,
                                  target_shape=(rgb.height, rgb.width),d_depth=list(rinfo.D),d_color=list(rinfo.D))
            else:
                d = reproject(d, list(dinfo.K), list(rinfo.K),
                              color_from_depth,
                              target_shape=(rgb.height, rgb.width),d_depth=list(dinfo.D),d_color=list(rinfo.D))
            if np.sum(np.isfinite(d)&(d>.2)&(d<8.))<10:
                rejected['insufficient_valid_depth']+=1; continue
            if rgb.encoding not in ('rgb8', 'bgr8') or rgb.step < rgb.width*3:
                raise ValueError('Unsupported RGB encoding/stride: ' + rgb.encoding)
            img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.step)
            img = img[:, :rgb.width*3].reshape(rgb.height, rgb.width, 3)
            if rgb.encoding == 'bgr8': img = img[:, :, ::-1]
            out = os.path.join(args.outdir, '%04d' % report['frames']); os.makedirs(out)
            Image.fromarray(img).save(os.path.join(out, 'rgb.png'))
            np.save(os.path.join(out, 'depth.npy'), d)
            meta = dict(schema_version=2, k=list(rinfo.K), aligned=aligned, frame=color_frame,
                source_depth_frame=frame, world_frame=world, root=world,
                stamp=entry['header'], depth_stamp=pair['header'], rgb_record=entry['record'],
                depth_record=pair['record'], pair_delta_ms=(pair['header']-entry['header'])*1000,
                t_world_optical=matrix, t_map_cam=matrix, geometry=tf.geometry,
                distortion_rgb=list(rinfo.D),distortion_depth=list(dinfo.D),
                bag=os.path.basename(entry['path']), depth_bag=os.path.basename(pair['path']))
            with open(os.path.join(out, 'meta.json'), 'w') as f: json.dump(meta, f, indent=2)
            report['frames'] += 1
        if not report['frames']:
            raise ValueError('No usable RGB-D frames: ' + str(dict(rejected)))
        report['state'] = 'complete'
    except Exception as exc:
        report.update(state='failed', error=str(exc)); raise
    finally:
        for bag in bags.values(): bag.close()
        report['rejected'] = dict(rejected)
        with open(os.path.join(args.outdir, 'sampling.json'), 'w') as f: json.dump(report, f, indent=2)
    print('Exported %d frames in %s frame; rejected %s' % (report['frames'], world, dict(rejected)))


if __name__ == '__main__': main()

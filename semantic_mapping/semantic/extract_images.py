#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""从 bag 里抽图做标注数据集，使用本机 ROS 的 Python。

    python2 extract_images.py <bag或会话目录> <输出目录> [--fps 1] [--max 300]

按 fps 抽样，可选缩略图差分去重。裂缝标注推荐 --format png --diff 0，
保留细节。--time-source record 仅按 bag 记录时间抽图，不用于 RGB-D 同步。
images.json 保留原始 Header、记录时间、分卷和帧号，方便追溯。
"""
import argparse
import os
import sys
import json
import re
import math


import numpy as np
import rosbag
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from viewer_core import session_files
from sampling import HeaderIndex

RGB = '/camera/rgb/image_raw'



def to_array(msg):
    if msg.encoding not in ('rgb8','bgr8') or msg.step<msg.width*3:
        raise ValueError('Unsupported RGB encoding or stride')
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    arr=buf[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return arr[:,:,::-1] if msg.encoding=='bgr8' else arr


def thumb_diff(a, b):
    """两张图的缩略图平均绝对差，用来判断是不是近似重复。"""
    ta = np.asarray(Image.fromarray(a).resize((32, 24)), dtype=np.int16)
    tb = np.asarray(Image.fromarray(b).resize((32, 24)), dtype=np.int16)
    return float(np.abs(ta - tb).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='会话里任意一卷；同会话其他分卷自动纳入')
    ap.add_argument('outdir')
    ap.add_argument('--fps', type=float, default=1.0, help='每秒抽几张')
    ap.add_argument('--max', type=int, default=300)
    ap.add_argument('--jpeg', type=int, default=92)
    ap.add_argument('--format', choices=['jpg', 'png'], default='jpg')
    ap.add_argument('--time-source', choices=['header', 'record'], default='header',
                    help='抽样时钟；record 可用于 Header 回退的独立图片导出')
    ap.add_argument('--diff', type=float, default=1.5,
                    help='和上一张的缩略图差小于这个值就跳过（0=不去重）')
    args = ap.parse_args()
    if (args.fps<=0 or math.isnan(args.fps) or math.isinf(args.fps) or args.max<=0
            or not 1<=args.jpeg<=100 or args.diff<0 or math.isnan(args.diff) or math.isinf(args.diff)):
        ap.error('Invalid sampling/encoding limits')

    files = session_files(args.bag)
    if not files: ap.error('No bag sources')
    matched=re.match(r'^(.*_\d{8}_\d{6})_\d+\.bag$',os.path.basename(files[0]))
    stem=matched.group(1) if matched else os.path.basename(files[0])[:-4]
    if os.path.exists(args.outdir): ap.error('Output exists; use a new directory')
    os.makedirs(args.outdir)
    manifest=dict(schema_version=2,state='running',images=[],sources=files,
                  time_source=args.time_source, fps=args.fps, format=args.format,
                  max_images=args.max, diff=args.diff, topic=RGB)
    with open(os.path.join(args.outdir,'images.json'),'w') as f: json.dump(manifest,f)
    clock=HeaderIndex(); next_t=None; previous_header=None; previous_record=None

    step = 1.0 / args.fps
    kept = 0
    skipped = 0
    last = None
    regressions = []
    record_regressions = []
    seen = 0
    try:
        for path in files:
            with rosbag.Bag(path) as bag:
                for topic, msg, _t in bag.read_messages(topics=[RGB]):
                    stamp = msg.header.stamp.to_sec()
                    record = _t.to_sec()
                    seen += 1
                    if previous_header is not None and stamp < previous_header - 1e-6:
                        regressions.append(dict(bag=path,seq=msg.header.seq,
                            previous_header=previous_header,stamp=stamp,record_stamp=record))
                    previous_header = stamp
                    if args.time_source == 'header':
                        clock.add(stamp,record,path,msg.header.seq)
                        sample_time = stamp
                    else:
                        if previous_record is not None and record < previous_record - 1e-6:
                            # Independent images need no continuous pose clock. Restart
                            # sampling in a new epoch without changing source timestamps.
                            record_regressions.append(dict(bag=path,seq=msg.header.seq,
                                previous_record=previous_record,record_stamp=record))
                            next_t = None
                        sample_time = record
                    previous_record = record
                    if next_t is not None and sample_time < next_t:
                        continue
                    next_t = sample_time + step
                    img = to_array(msg)
                    if last is not None and args.diff > 0 and thumb_diff(last, img) < args.diff:
                        skipped += 1
                        continue
                    last = img
                    name = '%s_t%d_s%d_i%06d.%s' % (stem,msg.header.stamp.to_nsec(),
                                                               msg.header.seq,kept,args.format)
                    options = dict(quality=args.jpeg, optimize=True) if args.format == 'jpg' else {}
                    Image.fromarray(img).save(os.path.join(args.outdir,name),**options)
                    manifest['images'].append(dict(name=name,stamp=stamp,record_stamp=record,
                        stamp_ns=msg.header.stamp.to_nsec(),record_stamp_ns=_t.to_nsec(),
                        bag=path,seq=msg.header.seq,session_id=stem,width=msg.width,
                        height=msg.height,source_encoding=msg.encoding,
                        record_epoch=len(record_regressions)))
                    kept += 1
                    if kept >= args.max:
                        break
            if kept >= args.max:
                break
        if not kept:
            raise ValueError('No usable images were exported')
        manifest['state'] = 'complete'
    except BaseException as exc:
        manifest.update(state='failed',error=str(exc))
        raise
    finally:
        manifest.update(skipped_similar=skipped,frames_scanned=seen,
                        header_regressions=regressions,record_regressions=record_regressions,
                        limit_reached=kept >= args.max)
        with open(os.path.join(args.outdir,'images.json'),'w') as f:
            json.dump(manifest,f,indent=2)
    print('抽出 %d 张（跳过近似重复 %d 张）→ %s' % (kept, skipped, args.outdir))


if __name__ == '__main__':
    main()

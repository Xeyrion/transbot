#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""从 bag 里抽图做标注数据集（rosbag 只有 py2）。

    python2 extract_images.py <bag或会话目录> <输出目录> [--fps 1] [--max 300]

为什么不能直接抽所有帧：30 Hz 的包里一秒有 30 张几乎一样的图，标起来是灾难。
这里按 fps 抽样，再用缩略图差分去掉近似重复的帧，保证每张都有新内容。

输出 `<会话名>_<时间戳>.jpg`，文件名带时间戳，方便回头对回原包。
"""
import argparse
import os
import sys
import json
import re


if sys.version_info[0] != 2:
    # Melodic 的 rosbag 只有 py2 版本。放在其它 import 之前，否则
    # `import rosbag` 会先炸，报一个和真正原因无关的 ImportError。
    sys.exit('这个脚本要用 python2 跑：python2 %s ...' % sys.argv[0])

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
    ap.add_argument('--diff', type=float, default=1.5,
                    help='和上一张的缩略图差小于这个值就跳过（0=不去重）')
    args = ap.parse_args()
    if args.fps<=0 or args.max<=0 or not 1<=args.jpeg<=100 or args.diff<0:
        ap.error('Invalid sampling/encoding limits')

    files = session_files(args.bag)
    if not files: ap.error('No bag sources')
    matched=re.match(r'^(.*_\d{8}_\d{6})_\d+\.bag$',os.path.basename(files[0]))
    stem=matched.group(1) if matched else os.path.basename(files[0])[:-4]
    if os.path.exists(args.outdir): ap.error('Output exists; use a new directory')
    os.makedirs(args.outdir)
    manifest=dict(schema_version=1,state='running',images=[],sources=files)
    with open(os.path.join(args.outdir,'images.json'),'w') as f: json.dump(manifest,f)
    clock=HeaderIndex(); next_t=None

    step = 1.0 / args.fps
    n = 0
    kept = 0
    skipped = 0
    last = None
    for path in files:
        bag = rosbag.Bag(path)
        end = bag.get_end_time()
        for topic, msg, _t in bag.read_messages(topics=[RGB]):
            stamp = msg.header.stamp.to_sec()
            clock.add(stamp,_t.to_sec(),path,msg.header.seq)
            if next_t is not None and stamp < next_t:
                continue
            next_t = stamp + step
            img = to_array(msg)
            if last is not None and args.diff > 0 and thumb_diff(last, img) < args.diff:
                skipped += 1
                continue
            last = img
            name = '%s_t%d_s%d_i%06d.jpg' % (stem,int(round(stamp*1e9)),msg.header.seq,kept)
            Image.fromarray(img).save(os.path.join(args.outdir, name),
                                      quality=args.jpeg, optimize=True)
            manifest['images'].append(dict(name=name,stamp=stamp,bag=path,seq=msg.header.seq,session_id=stem))
            kept += 1
            n += 1
            if n >= args.max:
                break
        bag.close()
        if n >= args.max:
            break

    manifest.update(state='complete' if kept else 'failed',skipped_similar=skipped)
    with open(os.path.join(args.outdir,'images.json'),'w') as f: json.dump(manifest,f,indent=2)
    if not kept: raise ValueError('No usable images were exported')
    print('抽出 %d 张（跳过近似重复 %d 张）→ %s' % (kept, skipped, args.outdir))


if __name__ == '__main__':
    main()

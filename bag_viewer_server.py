#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Long-lived bag reader: keeps one bag open and answers seek requests.

The GUI (py3) can't import rosbag (Melodic's is py2-only), so this process stays
alive and serves frames over stdin/stdout:

    in   {"t": 12.34}                seek to 12.34 s into the bag
    out  {"t": 12.30, "rgb": "...ppm", "depth": "...pgm",
          "scan": [[x,y],...], "pose": [x,y,yaw], "err": null}

Frames are written to fixed paths (overwritten each seek) so the GUI can load
them right after reading the reply.

    python2 bag_viewer_server.py <bag>
"""
from __future__ import print_function
from i18n import tr

import json
import math
import os
import sys
import bisect
import shutil
import tempfile
import hashlib
from viewer_core import TransformIndex, session_files, association, transform_xy, mapped_window
from semantic_replay import SemanticReplay, discover_runs

import numpy as np
import rosbag
import rospy

OUT = os.environ.get('BAGVIEW_DIR', '/tmp/bagview')

TOPICS = ['/camera/rgb/image_raw', '/camera/depth/image_raw', '/scan', '/odom']


_indexes = {}

def first_in(bag, topic, t, window=.5):
    """Nearest record timestamp, with a bounded age; never silently reuse old frames."""
    if topic not in _indexes:
        conns = [c for c in bag._get_connections() if c.topic == topic]
        entries = list(bag._get_entries(conns))
        _indexes[topic] = ([e.time.to_sec() for e in entries], entries)
    times, entries = _indexes[topic]
    if not times:
        return None
    i = bisect.bisect_left(times, t)
    candidates = [j for j in (i-1, i) if 0 <= j < len(times)]
    j = min(candidates, key=lambda k: abs(times[k]-t))
    if abs(times[j]-t) > window:
        return None
    for _, msg, _ in bag.read_messages(topics=[topic], start_time=entries[j].time,
                                       end_time=entries[j].time):
        return msg
    return None


def send(obj):
    """写一行 JSON；GUI 关掉之后管道会断，这里吞掉 Broken pipe 返回 False。

    python 默认忽略 SIGPIPE，所以对端一消失，write 抛的是 IOError(EPIPE)
    而不是把进程杀掉——不接住就是满屏 traceback。
    """
    try:
        sys.stdout.write(json.dumps(obj) + '\n')
        sys.stdout.flush()
        return True
    except (IOError, OSError):
        return False


def laser_yaw(bag, default=0.0):
    """激光相对车体（base_link）的安装角，从包里的 /tf 读。

    这个值不是 0：三个包里 /base_link -> /laser 都是 -180°，也就是雷达的
    +x 指向车尾。不转的话，图上"上"是雷达的正前方而不是车头，整幅图会
    相对车体转半圈。
    """
    try:
        for _, msg, _ in bag.read_messages(topics=['/tf']):
            for tr in msg.transforms:
                if tr.child_frame_id.strip('/') == 'laser':
                    q = tr.transform.rotation
                    return math.degrees(math.atan2(
                        2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y * q.y + q.z * q.z)))
    except Exception:
        pass
    return default


class PoseIndex(TransformIndex):
    def __init__(self, paths, skip_original_map=False, bounds=None, trajectory=None):
        TransformIndex.__init__(self)
        identity = [(os.path.abspath(p), os.path.getsize(p), os.path.getmtime(p)) for p in paths]
        key = hashlib.sha256(json.dumps([3, identity,skip_original_map,bounds,trajectory]).encode('utf-8')).hexdigest()
        cache_dir = os.environ.get('BAGTOOLS_CACHE_DIR', os.path.expanduser('~/.cache/transbot_bagtools'))
        cache = os.path.join(cache_dir, key+'.tf.json')
        self.cache_hit = False
        try:
            with open(cache) as f:
                saved = json.load(f)
            self.dynamic = {(a,b): values for a,b,values in saved['dynamic']}
            self.static = {(a,b): value for a,b,value in saved['static']}
            self.finish()
            self.cache_hit = True
            return
        except (IOError, ValueError, KeyError, TypeError):
            pass
        for path in paths:
            with rosbag.Bag(path, 'r') as source:
                for topic, msg, rec in source.read_messages(topics=['/tf', '/tf_static']):
                    if bounds and topic!='/tf_static' and ((bounds[0] is not None and rec.to_sec()<bounds[0]) or
                                                          (bounds[1] is not None and rec.to_sec()>bounds[1])):continue
                    for tr in msg.transforms:
                        if skip_original_map and path!=trajectory and (tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/'))==('map','odom'):continue
                        q = tr.transform.rotation
                        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                        self.add(tr.header.frame_id, tr.child_frame_id,
                                 tr.header.stamp.to_sec(),
                                 (tr.transform.translation.x, tr.transform.translation.y, yaw),
                                 static=topic == '/tf_static')
        self.finish()
        # Repeated fixed mounting transforms need only one value per session.
        for pair, values in list(self.dynamic.items()):
            if len(values) > 1 and all(v[1] == values[0][1] for v in values):
                self.static[pair] = values[0][1]
                del self.dynamic[pair]
        try:
            if not os.path.isdir(cache_dir):
                os.makedirs(cache_dir)
            fd, temporary = tempfile.mkstemp(dir=cache_dir, suffix='.tmp')
            with os.fdopen(fd, 'w') as f:
                json.dump(dict(dynamic=[(a,b,v) for (a,b),v in self.dynamic.items()],
                               static=[(a,b,v) for (a,b),v in self.static.items()]), f)
            os.rename(temporary, cache)
        except OSError:
            pass


def write_ppm(path, arr):
    h, w = arr.shape[:2]
    with open(path, 'wb') as f:
        f.write('P6\n%d %d\n255\n' % (w, h))
        f.write(arr.tobytes())


def write_pgm(path, arr):
    h, w = arr.shape[:2]
    with open(path, 'wb') as f:
        f.write('P5\n%d %d\n255\n' % (w, h))
        f.write(arr.tobytes())


def reader_context(path, start, end, linked, warning=None):
    t0, t1 = start, end
    map_note=None
    replaces_map=not linked or bool(linked.get('replace_map_odom'))
    bounds=linked.get('record_bounds') if linked and replaces_map else None
    t0,t1,usable,trimmed=mapped_window(t0,t1,linked)
    if not usable:
        linked=None
        warning=tr('此分卷不在新地图有效时段内，显示原始里程计视图。')
    elif trimmed:
        map_note=tr('新地图仅覆盖连续片段，时间轴已限制到该片段。')
    # The reconstruction already recorded the complete TF stream; reading it
    # avoids decompressing gigabytes of image chunks just to find transforms.
    trajectory = linked.get('trajectory') if linked else None
    paths = [trajectory] if trajectory else session_files(path)
    poses = PoseIndex(paths,skip_original_map=replaces_map,bounds=bounds if usable else None,trajectory=trajectory)
    if trajectory and (poses.lookup('odom', 'base_link', t0) is None or
                       poses.lookup('base_link', 'laser', t0) is None):
        poses = PoseIndex(session_files(path)+[trajectory],skip_original_map=replaces_map,
                          bounds=bounds,trajectory=trajectory)
    has_map = any(k[0] == 'map' for k in poses.dynamic) or any(k[0] == 'map' for k in poses.static)
    world = 'map' if has_map else ('odom' if any('odom' in k for k in set(poses.dynamic)|set(poses.static)) else 'base')
    mount = poses.lookup('base_link', 'laser', t0)
    yaw_deg = math.degrees(mount[2]) if mount else 0.
    map_info = None
    if linked:
        import yaml
        with open(linked['yaml']) as f:
            map_info = yaml.safe_load(f)
        image = map_info['image']
        if not os.path.isabs(image):
            map_info['image'] = os.path.join(os.path.dirname(linked['yaml']), image)
        elif not os.path.isfile(image):
            # map_saver 在机器人上写的是绝对路径（/home/jetson/bags/...），回传后
            # 栅格就躺在 yaml 旁边。照原路径找不到时退回同名文件，否则地图背景
            # 会静默加载失败，只剩一片黑。
            beside = os.path.join(os.path.dirname(linked['yaml']), os.path.basename(image))
            if os.path.isfile(beside):
                map_info['image'] = beside
    info = {'dur': t1 - t0, 't0': t0, 'bag': os.path.basename(path),
            'laser_yaw': round(yaw_deg, 1), 'has_map': has_map, 'world': world,
            'map_info': map_info, 'warning': warning, 'map_note': map_note,
            'tf_cache_hit': poses.cache_hit}
    return dict(info=info, poses=poses, t0=t0, t1=t1, world=world)


def main():
    global OUT
    OUT = tempfile.mkdtemp(prefix='bagview-')
    path = sys.argv[1]
    if not os.path.isdir(OUT):
        os.makedirs(OUT)
    bag = rosbag.Bag(path, 'r')
    t0, t1 = bag.get_start_time(), bag.get_end_time()

    warning = None
    try:
        linked = association(path)
        if linked and not os.path.isfile(linked['yaml']):
            raise ValueError(tr('关联地图文件缺失'))
        # 录制时存下的栅格没有单独的轨迹包，轨迹就在源包里
        if linked and linked.get('trajectory') and not os.path.isfile(linked['trajectory']):
            raise ValueError(tr('关联轨迹文件缺失'))
    except Exception as exc:
        warning = str(exc)
        linked = None
    raw_context = reader_context(path, t0, t1, linked, warning)
    context = raw_context
    semantic = None
    semantic_path = None
    contexts = {None: (context, None)}
    last_semantic_key = last_semantic_out = None
    info = dict(context['info'])
    info['semantic_runs'] = discover_runs(path)
    if not send(info):
        bag.close()
        return

    # 不能用 `for line in sys.stdin`：python2 的文件迭代带 8 KB 预读缓冲，
    # 管道不关就一直不返回，GUI 那边就永远等不到第一帧。readline 才是逐行。
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except Exception:
            continue
        if req.get('cmd') == 'close':
            break

        changed = False
        requested_path = os.path.abspath(req['semantic_run']) if req.get('semantic_run') else None
        if requested_path != semantic_path:
            try:
                if requested_path not in contexts:
                    candidate = SemanticReplay(requested_path, path)
                    candidate_context = reader_context(path, bag.get_start_time(), bag.get_end_time(), candidate.linked)
                    if candidate_context['world'] != candidate.world:
                        raise ValueError('Semantic world frame is unavailable in its recorded trajectory')
                    contexts[requested_path] = (candidate_context, candidate)
                context, semantic = contexts[requested_path]
                semantic_path = requested_path
                changed = True
            except Exception as exc:
                if not send(dict(err='Semantic result: '+str(exc), semantic_error=True,
                                 semantic_run=semantic_path)):
                    break
                continue
        t0, t1, poses, world = (context[k] for k in ('t0', 't1', 'poses', 'world'))
        t = float(req.get('record_time', t0+float(req.get('t', 0.0))))
        t = max(t0, min(t1, t))
        sample = semantic.nearest(t) if semantic else None
        if sample:
            t = sample['record']
        frame = world if changed else req.get('frame', world)
        frame = frame if frame in ('map', 'odom', 'base') else world
        sample_key = (semantic_path, sample['id'], frame) if sample else None
        if sample_key and not changed and sample_key == last_semantic_key:
            if not send(last_semantic_out):
                break
            continue
        want_map = frame != 'base'
        out = {'t': round(t-t0, 3), 'record_time': t, 'err': None, 'frame': frame,
               'stamps': {}, 'semantic_run': semantic_path}
        if changed:
            out['context'] = context['info']
        if semantic:
            out['semantic'] = semantic.payload(sample)

        try:
            if semantic:
                out['rgb'] = out['depth'] = None
                if sample:
                    meta = sample['meta']
                    out['rgb'] = os.path.join(sample['folder'], 'rgb.png')
                    out['stamps']['rgb'] = meta['stamp']-t0
                    out['stamps']['depth'] = meta['depth_stamp']-t0
                    key = hashlib.sha1(semantic_path.encode('utf-8')).hexdigest()[:12]
                    out['depth'] = os.path.join(OUT, 'depth_%s_%s.png' % (key, sample['id']))
                    if not os.path.isfile(out['depth']):
                        from PIL import Image
                        depth = np.load(os.path.join(sample['folder'], 'depth.npy'), allow_pickle=False)
                        valid = np.isfinite(depth) & (depth > 0)
                        grey = (255*(1-np.clip(np.nan_to_num(depth)/4., 0, 1))).astype(np.uint8)
                        rgb = np.repeat(grey[:, :, None], 3, axis=2)
                        rgb[~valid] = (90, 35, 100)  # Invalid depth, not a close surface.
                        Image.fromarray(rgb).save(out['depth'])
            else:
                msg = first_in(bag, '/camera/rgb/image_raw', t)
                if msg is not None and msg.encoding in ('bgr8', 'rgb8'):
                    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
                    arr = rows[:, :msg.width*3].reshape(msg.height, msg.width, 3)
                    if msg.encoding == 'bgr8':
                        arr = arr[:, :, ::-1]
                    p = os.path.join(OUT, 'rgb.ppm')
                    write_ppm(p, arr)
                    out['rgb'] = p
                    out['rgb_t'] = round(msg.header.stamp.to_sec() - t0, 3)
                else:
                    out['rgb'] = None

                if msg is not None:
                    out['stamps']['rgb'] = round(msg.header.stamp.to_sec()-t0, 3)
                msg = first_in(bag, '/camera/depth/image_raw', t)
                if msg is not None:
                    out['stamps']['depth'] = round(msg.header.stamp.to_sec()-t0, 3)
                    if msg.encoding.startswith('32F'):
                        dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
                        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
                        d = rows[:, :msg.width*4].copy().view(dtype).reshape(msg.height, msg.width)
                        d = np.nan_to_num(d)
                        v = np.clip(d / 4.0, 0, 1)
                    else:
                        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
                        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
                        d = rows[:, :msg.width*2].copy().view(dtype).reshape(msg.height, msg.width)
                        v = np.clip(d.astype(np.float32) / 4000.0, 0, 1)
                    p = os.path.join(OUT, 'depth.pgm')
                    write_pgm(p, (255 * (1 - v)).astype(np.uint8))
                    out['depth'] = p
                else:
                    out['depth'] = None

            msg = first_in(bag, '/scan', t)
            out['scan'] = None
            out['pose'] = None
            if msg is not None:
                stamp = msg.header.stamp.to_sec()
                out['stamps']['scan'] = round(stamp-t0, 3)
                target = frame if want_map else 'base_link'
                tf = poses.lookup(target, msg.header.frame_id, stamp)
                if tf is None:
                    raise ValueError('Missing/stale TF: %s <- %s at %.3f' %
                                     (target, msg.header.frame_id, stamp))
                mx, my, ma = tf
                r = np.asarray(msg.ranges, dtype=np.float32)
                ang = msg.angle_min + np.arange(len(r))*msg.angle_increment
                ok = np.isfinite(r) & (r > msg.range_min) & (r < min(msg.range_max, 6.0))
                x, y = (r*np.cos(ang))[ok], (r*np.sin(ang))[ok]
                # Raw laser points are transformed exactly once, including translation.
                xs, ys = transform_xy(tf, x, y)
                step = 1 if want_map else max(1, len(xs)//360)
                out['scan'] = [[round(float(x), 3), round(float(y), 3)]
                               for x,y in zip(xs[::step], ys[::step])]
                if want_map:
                    pose = poses.lookup(frame, 'base_link', stamp)
                    if pose is not None:
                        out['pose'] = [pose[0], pose[1], math.degrees(pose[2])]
            if not want_map:
                out['pose'] = [0., 0., 0.]
        except Exception as e:
            out['err'] = str(e)[:160]

        last_semantic_key = sample_key if not out.get('err') else None
        last_semantic_out = dict((k,v) for k,v in out.items() if k != 'context')
        if not send(out):
            break

    bag.close()
    shutil.rmtree(OUT, ignore_errors=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (IOError, OSError):
        pass

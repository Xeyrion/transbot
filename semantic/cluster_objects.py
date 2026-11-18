#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把同一个物体的多次观测聚成一个物体，输出 objects.json 和一张语义地图。

    python3 cluster_objects.py <frames_dir> [--radius 0.6] [--min-hits 3]

单帧投一个点可能偏二十厘米，同一个物体从不同角度被看到十几次，聚起来之后
位置会稳得多 —— 这也是"多帧融合"的全部意义。
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from viewer_core import frame_metadata

COLORS = [(239, 68, 68), (56, 189, 248), (74, 222, 128), (245, 158, 11),
          (168, 85, 247), (236, 72, 153), (20, 184, 166), (250, 204, 21)]


def dist(a, b):
    return math.sqrt((a['x']-b['x'])**2+(a['y']-b['y'])**2+(a.get('z',0)-b.get('z',0))**2)


def compatible(a,b,radius,association='point'):
    if association=='instance':
        if a['frame']==b['frame'] and a.get('observation_id')!=b.get('observation_id'):
            # Separated co-visible boxes cannot vote for one instance.
            if 'bbox_xywh' not in a or 'bbox_xywh' not in b:
                raise ValueError('Instance association requires observation bounding boxes')
            ax,ay,aw,ah=a['bbox_xywh'];bx,by,bw,bh=b['bbox_xywh']
            area=max(0,min(ax+aw,bx+bw)-max(ax,bx))*max(0,min(ay+ah,by+bh)-max(ay,by))
            iou=float(area)/max(1,aw*ah+bw*bh-area)
            if iou<.5:return False
        if a.get('label')=='chair' and b.get('label')=='chair':
            # Chair back/seat height changes are allowed; horizontal neighbours
            # still need close support and may not violate co-visibility.
            return ((a['x']-b['x'])/radius)**2+((a['y']-b['y'])/radius)**2+((a['z']-b['z'])/.8)**2<=1.
    return dist(a,b)<=radius


def cluster(points, radius, association='point'):
    """Complete-link groups: every pair stays within radius; no transitive bridges."""
    if radius <= 0:
        raise ValueError('radius must be positive')
    used = [False] * len(points)
    groups = []
    for i in range(len(points)):
        if used[i]:
            continue
        group = [i]
        used[i] = True
        changed = True
        while changed:
            changed = False
            for j in range(len(points)):
                if used[j]:
                    continue
                if all(compatible(points[j],points[k],radius,association) for k in group):
                    group.append(j)
                    used[j] = True
                    changed = True
        groups.append(group)
    return groups


def view_support(points, min_duration=0., min_baseline=0., min_angle=0.):
    """Require temporal and viewpoint evidence when requested, independent of FPS."""
    duration = max(p['t'] for p in points)-min(p['t'] for p in points) if points else 0.
    baseline = angle = 0.
    if min_baseline or min_angle:
        if not all('camera_xyz' in p and 'view_direction' in p for p in points):
            return False, dict(reason='missing_view_metadata', duration_s=duration)
        for i, a in enumerate(points):
            for b in points[i+1:]:
                baseline=max(baseline, math.sqrt(sum((x-y)**2 for x,y in zip(a['camera_xyz'],b['camera_xyz']))))
                av,bv=a['view_direction'],b['view_direction']
                norm=math.sqrt(sum(v*v for v in av)*sum(v*v for v in bv))
                if norm>0:
                    angle=max(angle,math.degrees(math.acos(max(-1.,min(1.,sum(x*y for x,y in zip(av,bv))/norm)))))
    enough_view = not (min_baseline or min_angle) or (min_baseline>0 and baseline>=min_baseline) or (min_angle>0 and angle>=min_angle)
    return duration>=min_duration and enough_view, dict(duration_s=duration,baseline_m=baseline,angle_deg=angle)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frames_dir')
    ap.add_argument('--radius', type=float, default=0.6, help='同一物体的归并半径（米）')
    ap.add_argument('--min-hits', type=int, default=3, help='少于这么多次观测就丢掉')
    ap.add_argument('--min-duration', type=float, default=0.)
    ap.add_argument('--min-view-baseline', type=float, default=0.)
    ap.add_argument('--min-view-angle', type=float, default=0.)
    ap.add_argument('--association',choices=('point','instance'),default='point')
    ap.add_argument('--classes', default='',
                    help='只保留这些类别，逗号分隔（如 chair,dining table,laptop,tv）；'
                         '空=全部。COCO 在陌生场景会误检 airplane/hot dog 这类，白名单能直接滤掉')
    ap.add_argument('--out', default=None)
    ap.add_argument('--map', default=None,
                    help='地图 yaml（.mapping.yaml 或 map_*.yaml），画在物体底下当背景')
    ap.add_argument('--include-dynamic',action='store_true',help='Explicitly allow moving-class observations in a static object map')
    args = ap.parse_args()
    if args.radius <= 0 or args.min_hits < 1 or min(args.min_duration,args.min_view_baseline,args.min_view_angle)<0:
        ap.error('radius and min-hits must be positive')

    data = json.load(open(os.path.join(args.frames_dir, 'observations.json')))
    obs = data['observations']
    dynamic={'person','bicycle','car','motorcycle','bus','truck','train','boat','bird','cat','dog','horse','sheep','cow'}
    excluded_dynamic=[]
    if not args.include_dynamic:
        excluded_dynamic=[o for o in obs if o['label'] in dynamic]
        obs=[o for o in obs if o['label'] not in dynamic]
    wanted = [c.strip() for c in args.classes.split(',') if c.strip()]
    if wanted:
        before = len(obs)
        obs = [o for o in obs if o['label'] in wanted]
        print('类别白名单 %s：%d → %d 个观测' % (wanted, before, len(obs)))
    by_label = {}
    for o in sorted(obs,key=lambda o:(o['label'],-o['score'],o['frame'],o['x'],o['y'],o['z'])):
        if not all(math.isfinite(o[k]) for k in ['x','y','z','score']):
            raise ValueError('Nonfinite object observation')
        by_label.setdefault(o['label'], []).append(o)

    objects = []
    rejected_groups = []
    for label, items in sorted(by_label.items()):
        for group in cluster(items, args.radius,args.association):
            # One frame contributes at most one vote to an instance.
            per_frame = {}
            for i in group:
                item = items[i]
                old = per_frame.get(item['frame'])
                if old is None or item['score'] > old['score']:
                    per_frame[item['frame']] = item
            pts = [per_frame[k] for k in sorted(per_frame)]
            if len(pts) < args.min_hits:
                rejected_groups.append(dict(label=label,reason='insufficient_frames',frames=sorted(per_frame),observations=[p.get('observation_id') for p in pts]))
                continue
            supported, support=view_support(pts,args.min_duration,args.min_view_baseline,args.min_view_angle)
            if not supported:
                rejected_groups.append(dict(label=label,reason='insufficient_time_or_view',frames=sorted(per_frame),support=support,observations=[p.get('observation_id') for p in pts]))
                continue
            weights=[max(.01,p['score'])/max(.02,p.get('depth_quality',{}).get('sigma_m',.1))**2 for p in pts]
            total=sum(weights)
            xs = [p['x'] for p in pts]
            ys = [p['y'] for p in pts]
            cx, cy = sum(x*w for x,w in zip(xs,weights))/total,sum(y*w for y,w in zip(ys,weights))/total
            spread = max(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys))
            objects.append(dict(label=label, hits=len(pts),
                                x=round(cx, 2), y=round(cy, 2),
                                z=round(sum(p['z']*w for p,w in zip(pts,weights))/total, 2),
                                score=round(sum(p['score'] for p in pts) / len(pts), 2),
                                spread=round(spread, 2),
                                frames=sorted(set(p['frame'] for p in pts)),
                                view_support=support,
                                observations=[p.get('observation_id',str(p['frame'])) for p in pts],
                                position_semantics='weighted visible surface samples; not object centre'))
    objects.sort(key=lambda o: (o['label'], -o['hits']))
    for i,item in enumerate(objects): item['object_id']='object-%04d'%i

    out = args.out or os.path.join(args.frames_dir, 'objects.json')
    with open(out, 'w') as f:
        json.dump(dict(schema_version=2, world_frame=data.get('world_frame'),
                       geometry=data.get('geometry'),radius=args.radius, min_hits=args.min_hits,
                       excluded_dynamic=len(excluded_dynamic),fusion='complete_link_3d_frame_dedup',
                       association=args.association,
                       support_policy=dict(min_duration_s=args.min_duration,min_baseline_m=args.min_view_baseline,min_angle_deg=args.min_view_angle),
                       rejected_groups=rejected_groups,
                       frames=data['frames'], objects=objects), f, indent=1)

    print('%-12s %5s %7s %7s %7s %6s' % ('类别', '次数', 'x', 'y', '散布', '置信'))
    for o in objects:
        print('%-12s %5d %7.2f %7.2f %7.2f %6.2f'
              % (o['label'], o['hits'], o['x'], o['y'], o['spread'], o['score']))

    render(objects, args.frames_dir, os.path.join(args.frames_dir, 'semantic_map.png'), args.map)
    print('→ ' + out)
    print('→ ' + os.path.join(args.frames_dir, 'semantic_map.png'))


def draw_map(canvas, map_yaml, to_px, ppm):
    """把占用栅格贴到画布上，让物体有房间轮廓可参照。

    栅格的三档灰直接映射成深色主题：未知→底色，空闲→深蓝灰，占用→浅灰。
    按地图 origin/yaw/resolution 做逆仿射映射。
    """
    import math
    import yaml
    info = yaml.safe_load(open(map_yaml))
    ox, oy, yaw = info['origin']
    res = float(info['resolution'])
    img_path = info['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(map_yaml), img_path)
    if not os.path.isfile(img_path):
        img_path = os.path.join(os.path.dirname(map_yaml), os.path.basename(img_path))
    if not os.path.isfile(img_path):
        raise ValueError('Map image is missing: ' + img_path)
    pgm = Image.open(img_path).convert('L')
    arr = np.asarray(pgm)
    rgb = np.zeros(arr.shape + (3,), np.uint8)
    rgb[...] = (13, 17, 23)               # 未知
    rgb[arr > 240] = (31, 41, 55)         # 空闲
    rgb[arr < 40] = (229, 231, 235)       # 占用（墙）
    block = Image.fromarray(rgb)
    # Inverse affine mapping handles arbitrary map yaw and cropped view bounds.
    center=np.asarray(to_px(0.,0.)); ex=np.asarray(to_px(1.,0.))-center; ey=np.asarray(to_px(0.,1.))-center
    basis=np.column_stack([ex,ey]); inv=np.linalg.inv(basis)
    c,s=math.cos(yaw),math.sin(yaw)
    rot=np.array([[c,s],[-s,c]])
    linear=rot.dot(inv)/res
    offset=rot.dot(-inv.dot(center)-np.array([ox,oy]))/res
    affine=(linear[0,0],linear[0,1],offset[0]-.5,
            -linear[1,0],-linear[1,1],arr.shape[0]-.5-offset[1])
    warped=block.convert('RGBA').transform(canvas.size,Image.AFFINE,affine,resample=Image.NEAREST)
    canvas.paste(warped,(0,0),warped)
    return True


def render(objects, frames_dir, path, map_yaml=None):
    """俯视图：地图 + 轨迹 + 物体。范围按轨迹自动扩。"""
    poses = []
    for meta_path in frame_metadata(frames_dir):
        m = json.load(open(meta_path))['t_map_cam']
        poses.append((m[0][3], m[1][3]))
    if not poses and not objects:
        return
    xs = [p[0] for p in poses] + [o['x'] for o in objects]
    ys = [p[1] for p in poses] + [o['y'] for o in objects]
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    span = max(max(xs) - min(xs), max(ys) - min(ys), 4.0) + 3.0
    ppm = min(60.0, 900.0 / span)
    side = int(span * ppm)
    img = Image.new('RGB', (side, side), (13, 17, 23))
    d = ImageDraw.Draw(img)

    def to_px(x, y):
        return side / 2.0 + (x - cx) * ppm, side / 2.0 - (y - cy) * ppm

    if map_yaml:
        worlds = set()
        for p in frame_metadata(frames_dir):
            with open(p) as f:
                meta = json.load(f)
            worlds.add(meta.get('world_frame', meta.get('root')))
        if worlds != {'map'}:
            raise ValueError('A map background requires explicit map-frame observations')
        if draw_map(img, map_yaml, to_px, ppm):
            d = ImageDraw.Draw(img)      # 贴图后要重新取画笔
        else:
            raise ValueError('Map background could not be rendered (check origin/yaw)')

    for i in range(1, len(poses)):
        d.line([to_px(*poses[i - 1]), to_px(*poses[i])], fill=(70, 80, 95))
    if poses:
        px, py = to_px(*poses[0])
        d.ellipse([px - 3, py - 3, px + 3, py + 3], fill=(120, 130, 145))

    labels = sorted(set(o['label'] for o in objects))
    color_of = {lab: COLORS[i % len(COLORS)] for i, lab in enumerate(labels)}
    for o in objects:
        px, py = to_px(o['x'], o['y'])
        r = 6
        d.ellipse([px - r, py - r, px + r, py + r], fill=color_of[o['label']])
        d.text((px + r + 2, py - 6), '%s×%d' % (o['label'], o['hits']),
               fill=color_of[o['label']])

    d.text((8, 8), 'semantic map: %d objects, %.1f m across' % (len(objects), span),
           fill=(215, 215, 215))
    y = 26
    for lab in labels:
        d.rectangle([8, y, 18, y + 8], fill=color_of[lab])
        d.text((24, y - 2), lab, fill=(215, 215, 215))
        y += 16
    img.save(path)


if __name__ == '__main__':
    main()

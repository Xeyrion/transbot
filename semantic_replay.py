# -*- coding: utf-8 -*-
"""Read saved semantic runs without importing a detector or a ROS runtime (py2/3).

Boxes are tied to the actual sampled PNG, never painted on a nearby raw frame.
Map points are the completed run's full-session fusion, not online predictions.
"""
from __future__ import division
import bisect
import glob
import hashlib
import json
import os
from viewer_core import session_files


COLORS = {'fused': (74, 222, 128), 'depth': (56, 189, 248),
          'rejected': (251, 146, 60), 'excluded': (156, 163, 175)}


def read_json(path):
    with open(path) as f:
        return json.load(f)


def validate_sources(run, bag_path):
    files = session_files(bag_path)
    sources = run.get('sources', [])
    expected = {os.path.basename(s['path']): s for s in sources}
    if not files or len(expected) != len(sources) or set(expected) != set(os.path.basename(p) for p in files):
        raise ValueError('Semantic result belongs to a different or incomplete recording session')
    for p in files:
        s = expected[os.path.basename(p)]
        if os.path.getsize(p) != s['size'] or abs(os.path.getmtime(p)-s['mtime']) > .001:
            raise ValueError('Semantic source fingerprint changed: '+os.path.basename(p))


def discover_runs(bag_path, root):
    """Cheap metadata-only discovery. Full validation happens when selected."""
    found = []
    for path in sorted(glob.glob(os.path.join(root, '*', 'run.json'))):
        try:
            run = read_json(path)
            if run.get('state') != 'complete':
                continue
            validate_sources(run, bag_path)
            model = os.path.splitext(os.path.basename(run.get('model', {}).get('path') or 'model'))[0]
            title = run.get('display_label_en') if os.environ.get('BAGTOOLS_LANG','').startswith('en') else run.get('display_label')
            found.append(dict(path=os.path.dirname(path), label=title or model+' | '+os.path.basename(os.path.dirname(path))))
        except (IOError, OSError, ValueError, KeyError, TypeError):
            continue
    return found


class SemanticReplay(object):
    def __init__(self, folder, bag_path):
        self.folder = os.path.abspath(folder)
        self.run = read_json(os.path.join(self.folder, 'run.json'))
        if self.run.get('state') != 'complete':
            raise ValueError('Semantic run is not complete')
        validate_sources(self.run, bag_path)
        sampling = read_json(os.path.join(self.folder, 'sampling.json'))
        observations = read_json(os.path.join(self.folder, 'observations.json'))
        objects = read_json(os.path.join(self.folder, 'objects.json'))
        self.world = self.run['world_frame']
        if self.world not in ('map', 'odom'):
            raise ValueError('Unsupported semantic world frame')
        self.linked = self.run.get('geometry', {}).get('map')
        for data in (sampling, observations, objects):
            if data.get('world_frame') != self.world or data.get('geometry', {}).get('map') != self.linked:
                raise ValueError('Semantic sidecars use a different map or coordinate frame')
        if self.world == 'map' and not self.linked:
            raise ValueError('Semantic map provenance is missing')
        # Review uses the original geometry bundle, even if a newer active map exists.
        for path, expected in self.run.get('geometry_sha256', {}).items():
            digest = hashlib.sha256()
            with open(path, 'rb') as f:
                for block in iter(lambda: f.read(1024*1024), b''):
                    digest.update(block)
            if digest.hexdigest() != expected:
                raise ValueError('Semantic geometry has changed: '+path)
        self.objects = objects.get('objects', [])
        self.model = os.path.splitext(os.path.basename(self.run.get('model', {}).get('path') or 'model'))[0]
        self.classes = None
        for stage in self.run.get('stages', []):
            cmd = stage.get('command', [])
            if stage.get('name') == 'fuse_render' and '--classes' in cmd:
                self.classes = set(cmd[cmd.index('--classes')+1].split(','))
        fused = {}
        for i, obj in enumerate(self.objects):
            obj['number'] = i+1
            for oid in obj.get('observations', []):
                fused[oid] = i+1
        obs = {o['observation_id']: o for o in observations.get('observations', [])}
        self.detections = {}
        for raw in observations.get('detections', []):
            det = dict(raw)
            oid = det.get('observation_id')
            det['included'] = self.classes is None or det['label'] in self.classes
            det['object_number'] = fused.get(oid)
            det['status'] = ('excluded' if not det['included'] else
                             'fused' if oid in fused else
                             'depth' if det.get('projection') == 'accepted' else 'rejected')
            if oid in obs:
                det['depth_m'] = obs[oid]['dist']
            self.detections.setdefault(det['source_frame'], []).append(det)
        self.samples = []
        for path in sorted(glob.glob(os.path.join(self.folder, '[0-9]*', 'meta.json'))):
            meta = read_json(path)
            if meta.get('bag') != os.path.basename(bag_path):
                continue
            if meta.get('world_frame') != self.world:
                raise ValueError('Sample coordinate frame differs from semantic run')
            folder = os.path.dirname(path)
            for name in ('rgb.png', 'depth.npy'):
                if not os.path.isfile(os.path.join(folder, name)):
                    raise ValueError('Missing semantic sample: '+os.path.join(folder, name))
            self.samples.append(dict(id=os.path.basename(folder), record=meta['rgb_record'],
                                     meta=meta, folder=folder))
        self.samples.sort(key=lambda s: s['record'])
        self.times = [s['record'] for s in self.samples]
        if not self.samples:
            raise ValueError('No semantic samples for this bag part')

    def nearest(self, record, max_age=.65):
        i = bisect.bisect_left(self.times, record)
        candidates = [j for j in (i-1, i) if 0 <= j < len(self.times)]
        j = min(candidates, key=lambda n: abs(self.times[n]-record))
        return self.samples[j] if abs(self.times[j]-record) <= max_age else None

    def payload(self, sample):
        return dict(model=self.model, run_id=self.run.get('run_id',os.path.basename(self.folder)), sample=sample['id'] if sample else None,
                    detections=self.detections.get(sample['id'], []) if sample else [],
                    objects=self.objects, world=self.world)


def label_size(draw, text):
    try:
        box = draw.textbbox((0, 0), text)
        return box[2]+4, box[3]+2
    except (AttributeError, ValueError):  # Older Pillow only supports TrueType in textbbox.
        w, h = draw.textsize(text)
        return w+4, h+2


def draw_detections(image, detections, show_other=False, labels=True):
    """Native image coordinates; PIL default font remains compatible with py2."""
    from PIL import ImageDraw
    image = image.convert('RGB').copy()
    if labels:
        from PIL import Image
        for det in detections:
            if det.get('mask') and (show_other or det.get('included',True)):
                with Image.open(det['mask']) as saved:
                    mask=saved.convert('L')
                if mask.size!=image.size:
                    raise ValueError('Instance mask and RGB image dimensions differ')
                alpha=mask.point(lambda v: int(v*.22))
                image.paste(Image.new('RGB',image.size,COLORS[det['status']]),(0,0),alpha)
    draw = ImageDraw.Draw(image)
    for det in detections:
        if not show_other and not det.get('included', True):
            continue
        x, y, w, h = det['bbox_xywh']
        color = COLORS[det['status']]
        for offset in (0, 1):
            draw.rectangle((x+offset, y+offset, x+w-offset, y+h-offset), outline=color)
        if labels:
            reasons={'insufficient_valid_depth':'few depth','mixed_depth':'mixed depth',
                     'insufficient_surface_support':'split depth','ambiguous_surfaces':'ambiguous',
                     'no_connected_surface':'no surface','missing_instance_mask':'no mask'}
            tag = ('#%d' % det['object_number'] if det.get('object_number') else
                   'D+' if det.get('projection') == 'accepted' else reasons.get(det.get('reason'),'no depth'))
            text = '%s %.2f %s' % (det['label'], det['score'], tag)
            if det.get('depth_m') is not None:
                text += ' %.2fm' % det['depth_m']
            width, height = label_size(draw, text)
            tx = max(0, min(x, image.width-width))
            ty = max(0, min(y-height-1, image.height-height))
            draw.rectangle((tx, ty, tx+width, ty+height), fill=(17, 17, 17))
            draw.text((tx+2, ty), text, fill=color)
    return image

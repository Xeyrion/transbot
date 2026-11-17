#!/usr/bin/env python3
"""One isolated, traceable offline run; never silently reuse old frame output."""
import argparse
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from viewer_core import session_files


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1048576), b''): h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('out', nargs='?')
    args = ap.parse_args()
    sources = session_files(os.path.abspath(args.bag))
    if not sources: ap.error('No bag sources found')
    for path in sources:
        if not os.path.isfile(path): ap.error('Missing bag: ' + path)
    if args.out:
        out = os.path.abspath(args.out)
        os.makedirs(out)  # Refuse pre-existing output, including an empty directory.
    else:
        parent = os.environ.get('BAGTOOLS_RESULTS_DIR', os.path.join(ROOT, 'results', 'semantic'))
        os.makedirs(parent, exist_ok=True)
        out = tempfile.mkdtemp(prefix=datetime.datetime.utcnow().strftime('run_%Y%m%d_%H%M%S_'), dir=parent)
    manifest = dict(schema_version=2, run_id=os.path.basename(out), state='running',
        sources=[dict(path=p, size=os.path.getsize(p), mtime=os.path.getmtime(p)) for p in sources],
        source_fingerprint_kind='size_mtime; not content checksum', stages=[],
        code_sha256={n:digest(os.path.join(HERE, n)) for n in
                     ['run_pipeline.py','extract_frames.py','sampling.py','pose.py','depth_projection.py','detect_project.py','cluster_objects.py']})
    def save():
        path = os.path.join(out, 'run.json')
        with open(path+'.tmp', 'w') as f: json.dump(manifest, f, indent=2)
        os.replace(path+'.tmp', path)
    def run(name, cmd, env=None):
        stage = dict(name=name, command=cmd, state='running'); manifest['stages'].append(stage); save()
        started = time.monotonic()
        try:
            subprocess.run(cmd, check=True, env=env)
            stage['state'] = 'complete'
        except (Exception, KeyboardInterrupt):
            stage['state'] = 'failed'; raise
        finally:
            stage['elapsed_s'] = time.monotonic()-started; save()
    try:
        backend = os.environ.get('BACKEND', 'yolo26')
        clean_env = dict(os.environ)
        clean_env.pop('PYTHONPATH', None); clean_env.pop('PYTHONHOME', None)
        if backend == 'yolo26':
            candidates = [os.path.expanduser(p) for p in
                ['~/miniconda3/envs/yolo/bin/python', '~/miniconda3/bin/python']]
            python = next((p for p in candidates if os.path.isfile(p)), None)
            if not python: raise ValueError('YOLO environment missing; explicitly select BACKEND=cv2 to use Darknet')
            weights = os.path.abspath(os.path.expanduser(os.environ.get('SEMANTIC_WEIGHTS', os.path.join(ROOT,'yolo26n.pt'))))
            if not os.path.isfile(weights): raise ValueError('Local weights missing: '+weights)
            clean_env['SEMANTIC_WEIGHTS'] = weights
            manifest['model'] = dict(backend=backend, path=weights, sha256=digest(weights), python=python)
        elif backend == 'cv2':
            python = sys.executable
            models = os.path.abspath(os.environ.get('SEMANTIC_MODELS', os.path.join(ROOT,'models')))
            clean_env['SEMANTIC_MODELS'] = models
            manifest['model'] = dict(backend=backend, directory=models,
                files={n:digest(os.path.join(models,n)) for n in ['yolov3-tiny.cfg','yolov3-tiny.weights','coco.names']})
        else:
            raise ValueError('Unsupported backend: '+backend)
        extract = ['python2', os.path.join(HERE,'extract_frames.py'), sources[0], out,
            '--step', os.environ.get('STEP','1'), '--max', os.environ.get('MAX_FRAMES','120'),
            '--max-pair-ms', os.environ.get('MAX_PAIR_MS','33')]
        for envkey, flag in [('WORLD_FRAME','--root'),('RECORD_START','--record-start'),
                             ('RECORD_END','--record-end'),('MAP_ASSOCIATION','--map-association')]:
            if os.environ.get(envkey): extract += [flag, os.environ[envkey]]
        if os.environ.get('BAGTOOLS_CALIBRATION'):
            config = os.path.abspath(os.path.expanduser(os.environ['BAGTOOLS_CALIBRATION']))
            snapshot = os.path.join(out,'calibration.json')
            shutil.copy2(config,snapshot)
            manifest['calibration_sha256'] = digest(snapshot)
            extract += ['--calibration',snapshot]
        if os.environ.get('ALLOW_ESTIMATED_EXTRINSICS') == '1': extract += ['--allow-estimated-extrinsics']
        if os.environ.get('OVERRIDE_RECORDED_EXTRINSICS') == '1': extract += ['--override-recorded-extrinsics']
        run('sample', extract)
        with open(os.path.join(out,'sampling.json')) as f: sampling = json.load(f)
        manifest.update(world_frame=sampling['world_frame'], geometry=sampling['geometry'])
        linked = sampling['geometry']['map']
        if linked:
            import yaml
            paths = [linked['yaml']]
            if linked.get('trajectory'): paths.append(linked['trajectory'])
            with open(linked['yaml']) as f: image = yaml.safe_load(f)['image']
            candidate = image if os.path.isabs(image) else os.path.join(os.path.dirname(linked['yaml']),image)
            if not os.path.isfile(candidate): candidate = os.path.join(os.path.dirname(linked['yaml']),os.path.basename(image))
            paths.append(candidate)
            manifest['geometry_sha256'] = {p:digest(p) for p in paths}
        run('detect', [python, os.path.join(HERE,'detect_project.py'), out, '--backend', backend,
                      '--projection',os.environ.get('PROJECTION','center'),
                      '--conf', os.environ.get('CONF','0.25')], env=clean_env)
        cluster = [sys.executable, os.path.join(HERE,'cluster_objects.py'), out,
            '--radius', os.environ.get('RADIUS','0.6'), '--min-hits', os.environ.get('MIN_HITS','3')]
        manifest['background'] = 'absent'
        if linked and sampling['world_frame'] == 'map':
            cluster += ['--map', linked['yaml']]; manifest['background'] = linked['yaml']
        if os.environ.get('CLASSES'): cluster += ['--classes', os.environ['CLASSES']]
        for envkey, flag in [('MIN_DURATION','--min-duration'),('MIN_VIEW_BASELINE','--min-view-baseline'),('MIN_VIEW_ANGLE','--min-view-angle'),('ASSOCIATION','--association')]:
            if os.environ.get(envkey):cluster += [flag,os.environ[envkey]]
        run('fuse_render', cluster)
        if any(os.path.getsize(s['path'])!=s['size'] or os.path.getmtime(s['path'])!=s['mtime'] for s in manifest['sources']):
            raise ValueError('Source bags changed during analysis')
        manifest['state'] = 'complete'; save()
        print('Completed: '+out)
        print('World frame: %s; background: %s' % (sampling['world_frame'],manifest['background']))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        manifest.update(state='failed', error=str(exc)); save()
        print('Run failed; evidence retained at '+out, file=sys.stderr)
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__': raise SystemExit(main())

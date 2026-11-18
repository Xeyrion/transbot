#!/usr/bin/env python3
"""One isolated, traceable offline run; never silently reuse old frame output."""
import argparse
import datetime
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
from viewer_core import association, session_files, semantic_directory
from semantic_presets import PRESETS, PRESET_ALIASES
from semantic_storage import digest, snapshot_map
from project_paths import models_dir


PROFILES = {key: (title, english, weights, projection) for key, title, english, weights, projection in PRESETS}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('out', nargs='?')
    ap.add_argument('--preset', choices=sorted(set(PROFILES) | set(PRESET_ALIASES)))
    ap.add_argument('--gallery', help='Override the recording-local PNG gallery with a shared export root')
    args = ap.parse_args()
    args.preset = PRESET_ALIASES.get(args.preset, args.preset)
    sources = session_files(os.path.abspath(args.bag))
    if not sources: ap.error('No bag sources found')
    if args.preset and any(glob_name.endswith('.bag.active') for glob_name in os.listdir(os.path.dirname(sources[0]))):
        ap.error('Recording is still active. Wait for the bag to close.')
    for path in sources:
        if not os.path.isfile(path): ap.error('Missing bag: ' + path)
    if args.out:
        out = os.path.abspath(args.out)
        os.makedirs(out)  # Refuse pre-existing output, including an empty directory.
    else:
        parent = os.environ.get('BAGTOOLS_RESULTS_DIR', semantic_directory(sources[0]))
        os.makedirs(parent, exist_ok=True)
        stamp = datetime.datetime.utcnow().strftime('%Y%m%d_%H%M%S')
        out = tempfile.mkdtemp(prefix='{}_{}_'.format(args.preset or 'run', stamp), dir=parent)
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
        settings = dict(os.environ)
        if args.preset:
            title, english, weights_name, projection = PROFILES[args.preset]
            manifest.update(preset=args.preset, display_label=title, display_label_en=english)
            settings.update(BACKEND='yolo26', SEMANTIC_WEIGHTS=os.path.join(models_dir(), weights_name),
                            PROJECTION=projection, CONF='0.35', STEP='1', MAX_PAIR_MS='33',
                            RADIUS='0.6', MIN_HITS='3', ASSOCIATION='point', MIN_DURATION='0',
                            MIN_VIEW_BASELINE='0', MIN_VIEW_ANGLE='0',
                            CLASSES='chair,dining table,laptop,tv,keyboard,mouse,bench')
            for key in ('MAP_ASSOCIATION', 'RECORD_START', 'RECORD_END', 'ALLOW_ESTIMATED_EXTRINSICS',
                        'OVERRIDE_RECORDED_EXTRINSICS'):
                settings.pop(key, None)
            calibration = os.path.join(os.path.dirname(sources[0]), 'calibration.json')
            if os.path.isfile(calibration):
                settings['BAGTOOLS_CALIBRATION'] = calibration
        backend = settings.get('BACKEND', 'yolo26')
        clean_env = dict(os.environ)
        clean_env.pop('PYTHONPATH', None); clean_env.pop('PYTHONHOME', None)
        if backend == 'yolo26':
            candidates = [os.path.expanduser(p) for p in
                ['~/miniconda3/envs/yolo/bin/python', '~/miniconda3/bin/python']]
            python = next((p for p in candidates if os.path.isfile(p)), None)
            if not python: raise ValueError('YOLO environment missing; explicitly select BACKEND=cv2 to use Darknet')
            weights = os.path.abspath(os.path.expanduser(settings.get('SEMANTIC_WEIGHTS', os.path.join(models_dir(),'yolo26s.pt'))))
            if not os.path.isfile(weights): raise ValueError('Local weights missing: '+weights)
            clean_env['SEMANTIC_WEIGHTS'] = weights
            manifest['model'] = dict(backend=backend, path=weights, sha256=digest(weights), python=python)
        elif backend == 'cv2':
            python = sys.executable
            models = os.path.abspath(os.environ.get('SEMANTIC_MODELS', models_dir()))
            clean_env['SEMANTIC_MODELS'] = models
            manifest['model'] = dict(backend=backend, directory=models,
                files={n:digest(os.path.join(models,n)) for n in ['yolov3-tiny.cfg','yolov3-tiny.weights','coco.names']})
        else:
            raise ValueError('Unsupported backend: '+backend)
        base_manifest = settings.get('MAP_ASSOCIATION')
        linked = association(sources[0], manifest=base_manifest)
        if args.preset:
            settings['WORLD_FRAME'] = 'map' if linked else 'odom'
        if linked:
            snapshot = snapshot_map(out, linked)
            if association(sources[0], manifest=base_manifest) != linked:
                raise ValueError('Base map changed while copying it. Retry after map building ends.')
            manifest['replay_map'] = snapshot
            snapshot_association = os.path.join(out, 'base_map_association.json')
            snapshot_link = dict(linked, yaml=snapshot['yaml'], trajectory=snapshot['trajectory'])
            with open(snapshot_association, 'w') as f:
                json.dump(snapshot_link, f, indent=2)
            settings['MAP_ASSOCIATION'] = snapshot_association
        extract = ['python2', os.path.join(HERE,'extract_frames.py'), sources[0], out,
            '--step', settings.get('STEP','1'), '--max-pair-ms', settings.get('MAX_PAIR_MS','33')]
        extract += ['--all'] if args.preset else ['--max', settings.get('MAX_FRAMES', '120')]
        for envkey, flag in [('WORLD_FRAME','--root'),('RECORD_START','--record-start'),
                             ('RECORD_END','--record-end'),('MAP_ASSOCIATION','--map-association')]:
            if settings.get(envkey): extract += [flag, settings[envkey]]
        if settings.get('BAGTOOLS_CALIBRATION'):
            config = os.path.abspath(os.path.expanduser(settings['BAGTOOLS_CALIBRATION']))
            snapshot = os.path.join(out,'calibration.json')
            shutil.copy2(config,snapshot)
            manifest['calibration_sha256'] = digest(snapshot)
            extract += ['--calibration',snapshot]
        if settings.get('ALLOW_ESTIMATED_EXTRINSICS') == '1': extract += ['--allow-estimated-extrinsics']
        if settings.get('OVERRIDE_RECORDED_EXTRINSICS') == '1': extract += ['--override-recorded-extrinsics']
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
                      '--projection',settings.get('PROJECTION','center'),
                      '--conf', settings.get('CONF','0.25')], env=clean_env)
        cluster = [sys.executable, os.path.join(HERE,'cluster_objects.py'), out,
            '--radius', settings.get('RADIUS','0.6'), '--min-hits', settings.get('MIN_HITS','3')]
        manifest['background'] = 'absent'
        if linked and sampling['world_frame'] == 'map':
            cluster += ['--map', linked['yaml']]; manifest['background'] = linked['yaml']
        if settings.get('CLASSES'): cluster += ['--classes', settings['CLASSES']]
        for envkey, flag in [('MIN_DURATION','--min-duration'),('MIN_VIEW_BASELINE','--min-view-baseline'),('MIN_VIEW_ANGLE','--min-view-angle'),('ASSOCIATION','--association')]:
            if settings.get(envkey):cluster += [flag,settings[envkey]]
        run('fuse_render', cluster)
        if any(os.path.getsize(s['path'])!=s['size'] or os.path.getmtime(s['path'])!=s['mtime'] for s in manifest['sources']):
            raise ValueError('Source bags changed during analysis')
        manifest['state'] = 'complete'; save()
        if args.gallery or args.preset:
            from map_display import export_final_map, gallery_image_path, export_recording_map
            try:
                if args.gallery:
                    export_final_map(out, gallery=args.gallery)
                    manifest['gallery_image'] = os.path.join(os.path.abspath(args.gallery), gallery_image_path(manifest, out))
                else:
                    manifest['gallery_image'] = export_recording_map(out, os.path.join(semantic_directory(sources[0]), 'gallery'))
            except (OSError, ValueError) as exc:
                manifest['gallery_error'] = str(exc)
                print('Gallery export failed: ' + str(exc), file=sys.stderr)
            save()
        print('Completed: '+out)
        print('World frame: %s; background: %s' % (sampling['world_frame'],manifest['background']))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        manifest.update(state='failed', error=str(exc)); save()
        print('Run failed; evidence retained at '+out, file=sys.stderr)
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__': raise SystemExit(main())

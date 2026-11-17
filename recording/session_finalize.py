#!/usr/bin/env python3
"""Keep container finalization, process outcome and indexed quality separate."""
import argparse
import glob
import json
import os
import subprocess
import sys
sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from calibration_schema import load_calibration
from session_status import last_result_path

REQUIRED = {
    'full': ['/camera/rgb/image_raw', '/camera/depth/image_raw',
             '/camera/rgb/camera_info', '/camera/depth/camera_info', '/scan', '/odom', '/tf'],
    'nav': ['/scan', '/odom', '/tf'],
}
EXPECTED_TYPES = dict(zip(REQUIRED['full'], ['sensor_msgs/Image', 'sensor_msgs/Image',
    'sensor_msgs/CameraInfo', 'sensor_msgs/CameraInfo', 'sensor_msgs/LaserScan',
    'nav_msgs/Odometry', 'tf2_msgs/TFMessage']))


def finalize(root, stamp, profile, rc, stopped=False, timed_out=False,
             stop_reason='natural', probe=None):
    prefix = os.path.join(root, 'cam_' + stamp)
    closed = sorted(glob.glob(prefix + '*.bag'))
    active = sorted(glob.glob(prefix + '*.bag.active'))
    container_ok = bool(closed) and not active and not timed_out
    process_ok = (rc == 0 or (stopped and rc in (130, 143))) and not timed_out
    process_ok = process_ok and stop_reason in ('natural', 'process_exit', 'requested', 'duration')
    counts, types, error = {}, {}, None
    if container_ok:
        try:
            if probe:
                inventory = probe(closed)
            else:
                result = subprocess.run(['python2', os.path.join(os.path.dirname(__file__),
                    'record_quality.py')] + closed, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE, timeout=60)
                if result.returncode:
                    raise ValueError(result.stderr.decode('utf-8', 'replace')[-500:])
                inventory = json.loads(result.stdout.decode('utf-8'))
            counts, types = inventory['topic_counts'], inventory['topic_types']
        except Exception as exc:
            error = str(exc)
    missing = [t for t in REQUIRED[profile] if counts.get(t, 0) <= 0]
    wrong_types = [t for t in REQUIRED[profile] if counts.get(t,0) and types.get(t) != [EXPECTED_TYPES[t]]]
    preflight = os.environ.get('BAGTOOLS_PREFLIGHT_OK', 'unknown')
    health_path=prefix+'.health.json'
    try:
        with open(health_path) as f: health=json.load(f)
    except (OSError,ValueError): health=dict(state='missing',faults=['observer_evidence_missing'])
    quality_ok = container_ok and not error and not missing and not wrong_types and preflight != '0' and health.get('state')=='healthy'
    data = dict(schema_version=2, session='cam_' + stamp, profile=profile,
        complete=container_ok and process_ok and quality_ok,
        container_finalized=container_ok, recorder_exit_ok=process_ok,
        recorder_exit_code=rc, stop_requested=stopped, stop_reason=stop_reason,
        finalize_timed_out=timed_out,
        preflight_status=preflight, allow_missing=os.environ.get('BAGTOOLS_ALLOW_MISSING', '0') == '1',
        data_quality=dict(status='passed' if quality_ok else 'failed',
            scope='indexed_topic_inventory; not geometric calibration or full decode',
            topic_counts=counts, topic_types=types, wrong_types=wrong_types, missing=missing, error=error),
        live_health=health,
        files=[dict(name=os.path.basename(p), size=os.path.getsize(p)) for p in closed+active])
    calibration = os.path.join(root, 'calibration.json')
    if os.path.isfile(calibration):
        config=load_calibration(calibration)
        data['calibration'] = dict(id=config['id'], estimated=config.get('estimated', False),
                                   file='calibration.json')
    with open(prefix + '.session.json.tmp', 'w') as f:
        json.dump(data, f, indent=2)
    os.replace(prefix + '.session.json.tmp', prefix + '.session.json')
    try:
        path=last_result_path()
        with open(path+'.tmp','w') as f:
            json.dump(dict(session=data['session'],directory=root,complete=data['complete'],stop_reason=data['stop_reason']),f)
        os.replace(path+'.tmp',path)
    except OSError:pass
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root'); ap.add_argument('stamp'); ap.add_argument('profile', choices=sorted(REQUIRED))
    ap.add_argument('rc', type=int); ap.add_argument('stopped', type=int)
    ap.add_argument('timed_out', type=int); ap.add_argument('reason')
    args = ap.parse_args()
    data = finalize(args.root, args.stamp, args.profile, args.rc, bool(args.stopped),
                    bool(args.timed_out), args.reason)
    print(json.dumps(data))
    return 0 if data['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

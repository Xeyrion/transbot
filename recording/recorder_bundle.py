#!/usr/bin/env python3
"""Build a complete recorder manifest for deployment and version-checked startup."""
import argparse
import hashlib
import json
from pathlib import Path

NAMES=['record_full.sh','auto_record_cam.sh','restart_camera.sh','preflight.sh','stop_recording.py',
       'session_status.py','session_finalize.py','record_quality.py','record_guard.py','publish_calibration.py',
       'live_health.py','stream_health.py','record_probe.py','calibration_schema.py','recorder_bundle.py','remote_start.py','check_tf.py','clock_check.py',
       'record_sources.launch','calibration.json']


def bundle():
    here=Path(__file__).resolve().parent; files={}
    for name in NAMES:
        path=here/name
        if name=='calibration_schema.py' and not path.exists(): path=here.parent/name
        files[name]=hashlib.sha256(path.read_bytes()).hexdigest()
    return files


if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--duration',type=int,default=300); ap.add_argument('--out',default='/home/jetson/bags')
    ap.add_argument('--name',default=''); ap.add_argument('--root',default='/home/jetson'); args=ap.parse_args()
    if args.duration<0: ap.error('Duration must be nonnegative')
    print(json.dumps(dict(protocol_version=1,files=bundle(),duration=args.duration,out=args.out,name=args.name,root=args.root)))

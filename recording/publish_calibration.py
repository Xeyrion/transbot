#!/usr/bin/env python3
"""Publish only explicitly configured installation transforms, never default guesses."""
import json
import math
import os
import subprocess
import sys
sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from calibration_schema import load_calibration


def commands(path):
    data=load_calibration(path)
    if not data.get('id'):
        raise ValueError('Calibration must have an id')
    allowed = {('base_footprint', 'base_link'), ('base_link', 'camera_link'),('base_link','laser')}
    result = []
    for item in data['transforms']:
        pair = (item['parent'], item['child'])
        if pair not in allowed:
            raise ValueError('Only fixed installation edges may be published: ' + str(pair))
        values = item['xyz'] + item['quaternion_xyzw']
        if len(values) != 7 or not all(math.isfinite(v) for v in values):
            raise ValueError('Invalid calibration values')
        if abs(sum(v*v for v in values[3:])-1.) > 1e-4:
            raise ValueError('Calibration quaternion must be normalized')
        result.append(['rosrun', 'tf2_ros', 'static_transform_publisher'] +
                      [str(v) for v in values] + list(pair))
    return data, result


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--camera-args':
        data, cmds = commands(sys.argv[2])
        camera = [cmd for cmd in cmds if cmd[-2:] == ['base_link', 'camera_link']]
        if len(camera) != 1:
            raise ValueError('Calibration must contain base_link -> camera_link')
        print(' '.join(camera[0][3:]))
        sys.exit(0)
    data, cmds = commands(sys.argv[1])
    if len(sys.argv) == 4:
        cmds = [cmd for cmd in cmds if cmd[-2:] == sys.argv[2:4]]
        if not cmds:
            raise ValueError('Requested installation transform is absent from calibration')
    print('Installation calibration:', data['id'], 'estimated=', data.get('estimated', False))
    with open('/tmp/transbot_calibration_publish.log', 'a') as log:
        for cmd in cmds:
            subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                             start_new_session=True)

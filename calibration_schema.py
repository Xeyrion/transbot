# -*- coding: utf-8 -*-
"""Shared installation-calibration contract, Python 2/3 and ROS-independent."""
import io
import json
import math
import numbers

ALLOWED_EDGES={('base_footprint','base_link'),('base_link','camera_link'),('base_link','laser')}


def load_calibration(path):
    with io.open(path,encoding='utf-8') as f: data=json.load(f)
    try: strings=(basestring,)
    except NameError: strings=(str,)
    if not isinstance(data.get('id'),strings) or not data['id'].strip(): raise ValueError('Calibration id is required')
    if data.get('schema_version',1)!=1: raise ValueError('Unsupported calibration schema')
    if 'estimated' in data and not isinstance(data['estimated'],bool): raise ValueError('estimated must be a JSON boolean')
    data.setdefault('estimated',True)
    seen=set()
    for item in data['transforms']:
        key=(item['parent'].strip('/'),item['child'].strip('/'))
        if key not in ALLOWED_EDGES or key in seen: raise ValueError('Invalid or duplicate installation edge: '+str(key))
        seen.add(key)
        xyz,q=item['xyz'],item['quaternion_xyzw']
        if len(xyz)!=3 or len(q)!=4: raise ValueError('Expected xyz[3], quaternion_xyzw[4]')
        if not all(isinstance(v,numbers.Real) and not isinstance(v,bool) and not math.isnan(v) and not math.isinf(v) for v in xyz+q):
            raise ValueError('Calibration contains nonfinite or nonnumeric values')
        if abs(sum(v*v for v in q)-1)>1e-4: raise ValueError('Quaternion must be normalized')
        item.update(parent=key[0],child=key[1])
    if not seen: raise ValueError('No installation transforms')
    return data

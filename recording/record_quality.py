#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Post-record indexed topic inventory; no ROS master or bag mutations."""
from __future__ import print_function
import json
import sys
import rosbag

counts = {}
types = {}
for path in sys.argv[1:]:
    with rosbag.Bag(path) as bag:
        for name, info in bag.get_type_and_topic_info()[1].items():
            counts[name] = counts.get(name, 0) + info.message_count
            types.setdefault(name, set()).add(info.msg_type)
print(json.dumps(dict(topic_counts=counts, topic_types={k:sorted(v) for k,v in types.items()})))

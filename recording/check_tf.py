#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""One-shot TF validation; no streaming tf_echo subprocess left in a pipe."""
import math
import sys
import rospy
import tf2_ros

rospy.init_node('bagtools_tf_check',anonymous=True)
buffer=tf2_ros.Buffer(); listener=tf2_ros.TransformListener(buffer)
try:
    tr=buffer.lookup_transform(sys.argv[1],sys.argv[2],rospy.Time(0),rospy.Duration(3)).transform
    p,q=tr.translation,tr.rotation
    values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    if any(math.isnan(v) or math.isinf(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>.01:
        raise ValueError('Invalid transform values')
except Exception as exc:
    sys.stderr.write(str(exc)+'\n'); raise SystemExit(1)

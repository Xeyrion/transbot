# -*- coding: utf-8 -*-
"""Shared presets for the reader and the offline runner."""
from __future__ import unicode_literals


# id, Chinese display name, English display name, shared weight file, projection
PRESETS = (
    ('yolo26s', 'YOLO26s', 'YOLO26s', 'yolo26s.pt', 'center'),
    ('yolo26s-seg', 'YOLO26s 分割', 'YOLO26s Segmentation', 'yolo26s-seg.pt', 'mask-surface'),
)
PRESET_ALIASES = {'b1': 'yolo26s-seg'}

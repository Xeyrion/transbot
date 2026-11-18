# -*- coding: utf-8 -*-
"""Header-time selection, shared by Python 2 readers and Python 3 tests."""
import bisect
import math


class ClockError(ValueError):
    pass


class HeaderIndex(object):
    def __init__(self):
        self.entries = []
        self.times = []

    def add(self, header, record, path, seq=0):
        if header <= 0 or math.isnan(header) or math.isinf(header):
            raise ClockError('Invalid sensor header timestamp')
        if self.times and header < self.times[-1] - 1e-6:
            raise ClockError('Header time regressed; select an audited record-time segment, do not sort across clock epochs')
        self.times.append(header)
        self.entries.append(dict(header=header, record=record, path=path, seq=seq))

    def nearest(self, stamp, max_delta):
        if not self.times:
            return None
        i = bisect.bisect_left(self.times, stamp)
        options = [j for j in (i-1, i) if 0 <= j < len(self.times)]
        j = min(options, key=lambda v: abs(self.times[v]-stamp))
        return self.entries[j] if abs(self.times[j]-stamp) <= max_delta else None

    def sampled(self, step):
        if step <= 0:
            raise ValueError('Sampling step must be positive')
        when = None
        for entry in self.entries:
            if when is None or entry['header'] >= when:
                yield entry
                when = entry['header'] + step

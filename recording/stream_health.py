# -*- coding: utf-8 -*-
"""Pure per-topic liveness/clock checks. Python 2/3; no ROS dependency."""
import math

TOPICS = {
    'full': ['/camera/rgb/image_raw','/camera/depth/image_raw','/scan','/odom','/tf'],
    'nav': ['/scan','/odom','/tf'],
}
LATCHED = ['/camera/rgb/camera_info','/camera/depth/camera_info']
ODOM_TF = '/tf:odom->base_footprint'


class StreamHealth(object):
    def __init__(self, profile, started, grace=8., timeout=3., clock_limit=.5):
        if any(math.isnan(v) or math.isinf(v) for v in (started,grace,timeout,clock_limit)) or grace<0 or timeout<=0 or clock_limit<=0:
            raise ValueError('Invalid stream timing thresholds')
        self.subscriptions = TOPICS[profile] + (LATCHED if profile == 'full' else [])
        self.required = self.subscriptions + [ODOM_TF]
        self.started, self.grace, self.timeout = started, grace, timeout
        self.clock_limit = clock_limit
        self.streams = {}
        self.fault = None
        self.events=[]

    def observe(self, topic, now, stamp=None):
        old = self.streams.get(topic)
        before_fault=self.fault
        if stamp is not None and topic not in LATCHED:
            if math.isnan(stamp) or math.isinf(stamp):
                self.fault = 'nonfinite_header:' + topic
            elif stamp <= 0:
                self.fault = 'zero_header:' + topic
            elif old and old.get('header') is not None and stamp < old['header'] - 1e-6:
                self.fault = 'header_regression:' + topic
            elif old and old.get('header') is not None and abs((stamp-old['header'])-(now-old['last_monotonic']))>self.clock_limit:
                self.fault = 'header_clock_jump:'+topic
        progress = old.get('last_progress_monotonic',now) if old else now
        if stamp is not None and (not old or old.get('header') is None or stamp>old['header']+1e-6):
            progress = now
        if self.fault!=before_fault and len(self.events)<20:
            self.events.append(dict(fault=self.fault,topic=topic,header=stamp,
                previous_header=old.get('header') if old else None,arrival_monotonic=now))
        self.streams[topic] = dict(last_monotonic=now,last_progress_monotonic=progress,
                                  header=stamp,count=(old['count'] if old else 0)+1)

    def snapshot(self, now):
        faults = [self.fault] if self.fault else []
        for topic in self.required:
            item = self.streams.get(topic)
            if item is None and now-self.started >= self.grace: faults.append('missing:' + topic)
            elif item and topic not in LATCHED and now-item['last_monotonic'] > self.timeout:
                faults.append('stale:' + topic)
            elif item and topic not in LATCHED and item.get('header') is not None and now-item['last_progress_monotonic']>self.clock_limit:
                faults.append('frozen_header:' + topic)
        if faults and self.fault is None: self.fault=faults[0]
        complete = all(t in self.streams for t in self.required)
        return dict(schema_version=1,state='failed' if faults else ('healthy' if complete else 'starting'),
                    started_monotonic=self.started,
                    faults=faults,streams={t:dict(v) for t,v in self.streams.items()},updated_monotonic=now,
                    events=[dict(v) for v in self.events],
                    scope='observed topic publication; not proof of bag delivery')

# -*- coding: utf-8 -*-
"""Conservative record-time candidate windows; never invent missing observations."""
import bisect
import math


def review(streams,max_gap=1.,margin=.5,pair_limit=.033):
    if max_gap<=0 or margin<0 or pair_limit<=0: raise ValueError('Invalid segmentation limits')
    if any(len(values)<2 for values in streams.values()): return dict(segments=[],issues=['missing_required_stream'])
    start=max(v[0][0] for v in streams.values()); end=min(v[-1][0] for v in streams.values())
    excluded=[]; issues=[]
    for topic,values in streams.items():
        for i,(rec,hdr) in enumerate(values):
            if not (math.isfinite(hdr) if hasattr(math,'isfinite') else not(math.isnan(hdr) or math.isinf(hdr))) or hdr<=0:
                excluded.append((rec-max_gap-margin,rec+max_gap+margin)); issues.append(dict(topic=topic,kind='invalid_header',record=rec)); continue
            if i:
                before,old=values[i-1]; delta=hdr-old
                if delta < -1e-6:
                    radius=max(max_gap,abs(delta))+margin
                    excluded.append((min(before,rec)-radius,max(before,rec)+radius))
                    issues.append(dict(topic=topic,kind='clock_regression',record=rec,delta=delta))
                elif delta>max_gap:
                    excluded.append((before-margin,rec+margin)); issues.append(dict(topic=topic,kind='time_gap',record=rec,delta=delta))
    merged=[]
    for lo,hi in sorted(excluded):
        if merged and lo<=merged[-1][1]: merged[-1][1]=max(hi,merged[-1][1])
        else: merged.append([lo,hi])
    windows=[]; current=start
    for lo,hi in merged:
        if hi<start or lo>end: continue
        if lo>current: windows.append((current,lo))
        current=max(current,hi)
    if current<end: windows.append((current,end))
    segments=[]
    for lo,hi in windows:
        lo+=margin; hi-=margin
        selected={t:[h for r,h in v if lo<=r<=hi] for t,v in streams.items()}
        if hi<=lo or any(len(v)<2 for v in selected.values()): continue
        if any(any(b<a or b-a>max_gap for a,b in zip(v,v[1:])) for v in selected.values()): continue
        row=dict(record_start=lo,record_end=hi,counts={t:len(v) for t,v in selected.items()},time_checks='passed')
        if 'rgb' in selected and 'depth' in selected:
            depth=selected['depth']; diffs=[]
            for stamp in selected['rgb']:
                i=bisect.bisect_left(depth,stamp)
                diffs.append(min(abs(depth[j]-stamp) for j in [i-1,i] if 0<=j<len(depth)))
            row['rgbd_within_limit_fraction']=sum(v<=pair_limit for v in diffs)/float(len(diffs))
            row['rgbd_p95_ms']=sorted(diffs)[int(.95*(len(diffs)-1))]*1000
        segments.append(row)
    return dict(segments=segments,issues=issues,excluded_record_intervals=merged,
                scope='time-only candidates; calibration, motion and mapping accuracy require separate checks')

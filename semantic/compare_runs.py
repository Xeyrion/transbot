#!/usr/bin/env python3
"""Controlled depth/fusion experiments sharing immutable RGB-D samples.

python3 semantic/compare_runs.py <completed-baseline-run> --suite depth_v1
Each completed variant is a normal viewer-compatible run. No active map changes.
"""
import argparse
import copy
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from collections import Counter

HERE=os.path.dirname(os.path.abspath(__file__))
ROOT=os.path.dirname(HERE)
sys.path.insert(0,ROOT)
from semantic_replay import replay_geometry, validate_sources
from semantic_storage import snapshot_map
from viewer_core import frame_metadata, semantic_directory
from run_pipeline import digest


def save(path,data):
    with open(path+'.tmp','w') as f:json.dump(data,f,indent=2,ensure_ascii=False)
    os.replace(path+'.tmp',path)


def link_samples(source,out):
    """Only sample assets are shared; derived JSON and predictions stay isolated."""
    for name in ('sampling.json','calibration.json'):
        if os.path.isfile(os.path.join(source,name)):shutil.copy2(os.path.join(source,name),out)
    identity=hashlib.sha256()
    for folder in map(os.path.dirname, frame_metadata(source)):
        dest=os.path.join(out,os.path.basename(folder));os.mkdir(dest)
        for name in ('meta.json','rgb.png','depth.npy'):
            path=os.path.join(folder,name);target=os.path.join(dest,name)
            try:os.link(path,target)
            except OSError:shutil.copy2(path,target)
            identity.update((os.path.basename(folder)+'/'+name+':'+digest(path)).encode('utf-8'))
    return identity.hexdigest()


def legacy_predictions(baseline,out):
    data=json.load(open(os.path.join(baseline,'observations.json')))
    frames={}
    for folder in map(os.path.dirname, frame_metadata(baseline)):
        meta=json.load(open(os.path.join(folder,'meta.json')))
        frames[os.path.basename(folder)]=dict(rgb_sha256=digest(os.path.join(folder,'rgb.png')),
            rgb_record=meta['rgb_record'],detections=[])
    for det in data['detections']:
        frames[det['source_frame']]['detections'].append({k:det[k] for k in ('label','score','bbox_xywh')})
    manifest=json.load(open(os.path.join(baseline,'run.json')))
    command=next(s['command'] for s in manifest['stages'] if s['name']=='detect')
    floor=float(command[command.index('--conf')+1])
    path=os.path.join(out,'baseline_predictions.json');save(path,dict(schema_version=1,confidence_floor=floor,frames=frames))
    return path


def statistics(folder):
    observations=json.load(open(os.path.join(folder,'observations.json')))
    objects=json.load(open(os.path.join(folder,'objects.json')))
    used=set(oid for obj in objects['objects'] for oid in obj['observations'])
    return dict(frames=observations['frames'],detections=dict(Counter(d['label'] for d in observations['detections'])),
                accepted=dict(Counter(o['label'] for o in observations['observations'])),
                rejected=dict(Counter(d.get('reason') for d in observations['detections'] if d['projection']=='rejected')),
                candidates=dict(Counter(o['label'] for o in objects['objects'])),
                accepted_not_fused=sum(o['observation_id'] not in used for o in observations['observations']),
                rejected_groups=dict(Counter(g['reason'] for g in objects.get('rejected_groups',[]))))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('baseline');ap.add_argument('--suite',required=True)
    ap.add_argument('--seg-weights',default=os.path.join(ROOT,'models','yolo26s-seg.pt'))
    ap.add_argument('--no-dense',action='store_true')
    args=ap.parse_args()
    baseline=os.path.abspath(args.baseline);base=json.load(open(os.path.join(baseline,'run.json')))
    if base['state']!='complete':raise ValueError('Baseline run is not complete')
    validate_sources(base,base['sources'][0]['path'])
    recording = semantic_directory(base['sources'][0]['path'])
    linked = replay_geometry(base, baseline)
    suite=os.path.join(recording,'comparisons',args.suite);os.makedirs(suite)
    python=base['model']['python'];seg=os.path.abspath(args.seg_weights)
    if not os.path.isfile(seg):raise ValueError('Segmentation weights missing: '+seg)
    clean=dict(os.environ);clean.pop('PYTHONPATH',None);clean.pop('PYTHONHOME',None)
    summary=dict(suite=args.suite,state='running',created_utc=datetime.datetime.utcnow().isoformat()+'Z',
                 baseline=baseline,source_session=os.path.dirname(base['sources'][0]['path']),
                 evaluation='diagnostics and visual comparison; no accuracy ground truth',variants=[])
    summary_path=os.path.join(suite,'comparison.json');save(summary_path,summary)
    cached=legacy_predictions(baseline,suite)
    classes=next(s['command'][s['command'].index('--classes')+1] for s in base['stages'] if s['name']=='fuse_render' and '--classes' in s['command'])

    def variant(code,label,label_en,source,projection,predictions=None,seg_model=False,
                radius=.6,view=False,observations_from=None,association='point'):
        out=os.path.join(recording,args.suite+'_'+code);os.mkdir(out)
        run=copy.deepcopy(base)
        run.pop('preset', None)
        run.pop('gallery_image', None)
        run.pop('gallery_error', None)
        run.update(run_id=os.path.basename(out),state='running',stages=[],display_label=code+' '+label,
                   display_label_en=code+' '+label_en,parent_run=baseline,
                   experiment=dict(suite=args.suite,projection=projection,radius=radius,view_gate=view,association=association),
                   code_sha256={n:digest(os.path.join(HERE,n)) for n in
                       ('compare_runs.py','depth_projection.py','detect_project.py','cluster_objects.py')})
        if seg_model:
            run['model']=dict(backend='yolo26',path=seg,sha256=digest(seg),python=python)
        if linked:
            run['replay_map'] = snapshot_map(out, linked)
        path=os.path.join(out,'run.json');save(path,run)
        print('START '+code+' '+label_en,flush=True)

        def stage(name,cmd,env=None):
            record=dict(name=name,command=cmd,state='running');run['stages'].append(record);save(path,run)
            start=time.monotonic()
            with open(os.path.join(out,name+'.log'),'w') as log:
                result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,env=env)
            record.update(state='complete' if result.returncode==0 else 'failed',elapsed_s=time.monotonic()-start);save(path,run)
            if result.returncode:raise RuntimeError(code+' '+name+' failed; see '+out+'/'+name+'.log')
        try:
            run['sample_fingerprint']=link_samples(source,out)
            run['sample_reuse']=dict(source=source,policy='immutable hardlinks; copy fallback')
            if observations_from:
                shutil.copy2(os.path.join(observations_from,'observations.json'),out)
                run['observation_reuse']=dict(source=observations_from,sha256=digest(os.path.join(out,'observations.json')))
            else:
                cmd=[python,os.path.join(HERE,'detect_project.py'),out,'--conf','.35','--projection',projection]
                if predictions:
                    cmd+=['--predictions',predictions]
                    run['prediction_reuse']=dict(path=predictions,sha256=digest(predictions))
                env=dict(clean,SEMANTIC_WEIGHTS=seg if seg_model else base['model']['path'])
                stage('detect',cmd,env)
            cmd=[sys.executable,os.path.join(HERE,'cluster_objects.py'),out,'--radius',str(radius),'--min-hits','3','--classes',classes]
            cmd+=['--association',association]
            if run['world_frame']=='map':cmd+=['--map',os.path.join(out, run['replay_map']['yaml'])]
            if view:cmd+=['--min-duration','1','--min-view-baseline','.15','--min-view-angle','8']
            stage('fuse_render',cmd)
            validate_sources(run,base['sources'][0]['path'])
            run['state']='complete';save(path,run)
            row=dict(code=code,path=out,label=label,label_en=label_en,statistics=statistics(out),
                     projection=projection,radius=radius,view_gate=view,association=association,sample_fingerprint=run['sample_fingerprint'])
            summary['variants'].append(row);save(summary_path,summary)
            print('DONE '+code+' '+json.dumps(row['statistics']),flush=True)
            return out
        except BaseException as exc:
            run.update(state='failed',error=str(exc));save(path,run);raise

    try:
        variant('A0','原框中心深度 1Hz','Box centre / 1 Hz',baseline,'center',cached)
        variant('A1','原框连通表面 1Hz','Box surface / 1 Hz',baseline,'surface',cached)
        b0=variant('B0','分割模型中心深度 1Hz','Seg model centre / 1 Hz',baseline,'center',seg_model=True)
        b1=variant('B1','掩膜连通表面 1Hz','Mask surface / 1 Hz',baseline,'mask-surface',os.path.join(b0,'predictions.json'),True)
        variant('B2','掩膜较紧融合 1Hz','Mask surface / radius .45 m',baseline,'mask-surface',seg_model=True,radius=.45,observations_from=b1)
        if not args.no_dense:
            dense=os.path.join(suite,'samples_3hz')
            command=list(next(s['command'] for s in base['stages'] if s['name']=='sample'))
            command[2:4]=[base['sources'][0]['path'],dense]
            if linked:
                map_manifest = os.path.join(suite, 'base_map_association.json')
                save(map_manifest, linked)
                if '--map-association' in command:
                    command[command.index('--map-association')+1]=map_manifest
                else:
                    command += ['--map-association', map_manifest]
            if '--calibration' in command:
                command[command.index('--calibration')+1]=os.path.join(baseline,'calibration.json')
            command[command.index('--step')+1]=str(1./3)
            if '--all' in command:
                command.remove('--all')
                command += ['--max', '1000']
            else:
                command[command.index('--max')+1]='1000'
            with open(os.path.join(suite,'sample_3hz.log'),'w') as log:
                subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT)
            # Frames outside the original complete recording are never added.
            c0=variant('C0','掩膜连通表面 3Hz','Mask surface / 3 Hz',dense,'mask-surface',seg_model=True)
            variant('C1','3Hz 时间与视角验证','3 Hz / temporal + viewpoint support',dense,'mask-surface',seg_model=True,view=True,observations_from=c0)
            variant('C2','3Hz 实例约束融合','3 Hz / co-visible instance association',dense,'mask-surface',seg_model=True,radius=.45,view=True,observations_from=c0,association='instance')
        summary['state']='complete';save(summary_path,summary)
    except BaseException as exc:
        summary.update(state='failed',error=str(exc));save(summary_path,summary);raise
    print('COMPARISON '+summary_path,flush=True)


if __name__=='__main__':main()

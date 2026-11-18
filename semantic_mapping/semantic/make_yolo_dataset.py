#!/usr/bin/env python3
"""Validated, non-destructive YOLO export with session-separated validation."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import shutil


def validate_label(path,classes):
    for number,line in enumerate(path.read_text(encoding='utf-8').splitlines(),1):
        if not line.strip(): continue
        values=line.split()
        if len(values)!=5: raise ValueError('%s:%d requires class cx cy w h'%(path,number))
        cls=int(values[0]); x,y,w,h=map(float,values[1:])
        if cls<0 or cls>=classes or not all(math.isfinite(v) for v in [x,y,w,h]):
            raise ValueError('Invalid class or numeric label: '+str(path))
        if w<=0 or h<=0 or min(x-w/2,y-h/2)<-1e-6 or max(x+w/2,y+h/2)>1+1e-6:
            raise ValueError('Bounding box extends outside image: '+str(path))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('src',type=Path); ap.add_argument('out',type=Path)
    ap.add_argument('--names',required=True); ap.add_argument('--val',type=float,default=.2)
    ap.add_argument('--seed',type=int,default=0); ap.add_argument('--split-mode',choices=['session','image'],default='session')
    ap.add_argument('--group-map',type=Path,help='JSON mapping relative image names to session IDs')
    a=ap.parse_args(); names=[s.strip() for s in a.names.split(',')]
    if not names or any(not s for s in names) or len(set(names))!=len(names): ap.error('Class names must be nonempty and unique')
    if not 0<a.val<1: ap.error('val must be between zero and one')
    if a.out.exists(): ap.error('Output exists; use a new directory')
    if a.src.resolve()==a.out.resolve() or a.src.resolve() in a.out.resolve().parents:
        ap.error('Output must be outside the source dataset')
    groups={}
    if a.group_map: groups=json.loads(a.group_map.read_text(encoding='utf-8'))
    for manifest in a.src.rglob('images.json'):
        data=json.loads(manifest.read_text(encoding='utf-8'))
        if data.get('state')!='complete': ap.error('Incomplete image extraction: '+str(manifest))
        for item in data['images']:
            key=str((manifest.parent/item['name']).relative_to(a.src)).replace('\\','/')
            groups.setdefault(key,item['session_id'])
    entries=[]; hashes={}
    for image in sorted(p for p in a.src.rglob('*') if p.suffix.lower() in ['.jpg','.jpeg','.png']):
        label=image.with_suffix('.txt')
        if not label.exists(): continue
        validate_label(label,len(names))
        from PIL import Image
        with Image.open(str(image)) as check_image: check_image.verify()
        relative=str(image.relative_to(a.src)).replace('\\','/')
        group=groups.get(relative)
        if group is None and image.parent!=a.src: group=str(image.parent.relative_to(a.src))
        if group is None and a.split_mode=='session': ap.error('Session identity is missing for '+relative+'; provide --group-map')
        group=group if a.split_mode=='session' else relative
        if not isinstance(group,str) or not group.strip(): ap.error('Invalid session identity: '+relative)
        digest=hashlib.sha256(image.read_bytes()).hexdigest()
        if digest in hashes and hashes[digest]!=group:
            ap.error('Identical images appear across proposed groups: '+relative)
        hashes[digest]=group
        entries.append(dict(path=image,label=label,relative=relative,group=group,sha256=digest,
                            label_sha256=hashlib.sha256(label.read_bytes()).hexdigest()))
    unique=sorted(set(e['group'] for e in entries))
    if len(unique)<2: ap.error('At least two independent groups are required for train/val')
    random.Random(a.seed).shuffle(unique)
    val=set(unique[:max(1,min(len(unique)-1,round(len(unique)*a.val)))])
    a.out.mkdir(parents=True)
    records=[]
    for item in entries:
        split='val' if item['group'] in val else 'train'
        filename=hashlib.sha256(item['relative'].encode('utf-8')).hexdigest()[:16]+item['path'].suffix.lower()
        for sub in ['images','labels']: (a.out/split/sub).mkdir(parents=True,exist_ok=True)
        shutil.copy2(str(item['path']),str(a.out/split/'images'/filename))
        shutil.copy2(str(item['label']),str(a.out/split/'labels'/Path(filename).with_suffix('.txt')))
        if hashlib.sha256((a.out/split/'images'/filename).read_bytes()).hexdigest()!=item['sha256'] or hashlib.sha256((a.out/split/'labels'/Path(filename).with_suffix('.txt')).read_bytes()).hexdigest()!=item['label_sha256']:
            raise ValueError('Source annotation/image changed during dataset export')
        records.append(dict(source=item['relative'],group=item['group'],split=split,sha256=item['sha256'],output=filename))
    # JSON flow values are valid YAML and safely quote spaces, colons and apostrophes.
    (a.out/'data.yaml').write_text('path: '+json.dumps(str(a.out.resolve()))+'\ntrain: train/images\nval: val/images\nnames: '+json.dumps(names)+'\n',encoding='utf-8')
    (a.out/'split.json').write_text(json.dumps(dict(schema_version=1,mode=a.split_mode,seed=a.seed,
        warning='image split is not a generalization evaluation' if a.split_mode=='image' else None,
        records=records),indent=2),encoding='utf-8')
    print('Dataset ready:',a.out)


if __name__=='__main__': main()

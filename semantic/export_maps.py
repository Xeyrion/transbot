#!/usr/bin/env python3
"""Export completed runs into one easy-to-find standalone PNG gallery."""
import argparse
import glob
import json
import os
import sys

ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)));sys.path.insert(0,ROOT)
from map_display import export_final_map, gallery_image_path


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',default=os.path.join(ROOT,'results','semantic'))
    ap.add_argument('--out',default=os.path.join(ROOT,'results','semantic_maps'));args=ap.parse_args()
    os.makedirs(args.out,exist_ok=True);rows=[]
    for path in sorted(glob.glob(os.path.join(args.root,'*','run.json'))):
        with open(path) as f:run=json.load(f)
        if run.get('state')!='complete':continue
        folder=os.path.dirname(path);export_final_map(folder,gallery=args.out)
        title=run.get('display_label_en') or run.get('run_id',os.path.basename(folder))
        name=gallery_image_path(run,folder).replace(os.sep, '/')
        recording=name.split('/', 1)[0]
        rows.append(dict(recording=recording,title=run.get('display_label',title),image=name,run=folder,
                         world=run['world_frame'],sources=[source['path'] for source in run.get('sources', [])]))
        print(name)
    with open(os.path.join(args.out,'index.json'),'w',encoding='utf-8') as f:
        json.dump(rows,f,indent=2,ensure_ascii=False)
    with open(os.path.join(args.out,'README.md'),'w',encoding='utf-8') as f:
        f.write('# 语义地图图库\n\n每个子目录对应一次录制。同次录制的分卷共用一个子目录，目录内每张 PNG 对应一个算法版本。\n')
        for recording in sorted({row['recording'] for row in rows}):
            group=[row for row in rows if row['recording']==recording]
            f.write('\n## '+recording+'\n\n')
            for path in sorted({path for row in group for path in row['sources']}):
                f.write('- 来源 bag `'+os.path.basename(path.replace('\\', '/'))+'`\n')
            f.write('\n')
            for row in group:
                f.write('- [%s](%s)（%s）\n'%(row['title'],row['image'],row['world']))
    print('Gallery: '+os.path.abspath(args.out))


if __name__=='__main__':main()

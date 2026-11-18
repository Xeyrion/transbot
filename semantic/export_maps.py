#!/usr/bin/env python3
"""Export completed runs into one easy-to-find standalone PNG gallery."""
import argparse
import glob
import json
import os
import sys

ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)));sys.path.insert(0,ROOT)
from map_display import export_final_map, export_recording_map, gallery_image_path
from viewer_core import semantic_directory


def main():
    ap = argparse.ArgumentParser()
    inputs = ap.add_mutually_exclusive_group()
    inputs.add_argument('--bag', help='Export one recording')
    inputs.add_argument('--root', help='Read runs from an explicit directory')
    ap.add_argument('--out', help='Export an extra shared gallery with its own index')
    args = ap.parse_args()
    if args.root or args.bag:
        root = args.root or semantic_directory(args.bag)
        paths = glob.glob(os.path.join(root, '*', 'run.json'))
    else:
        paths = glob.glob(os.path.join(ROOT, 'bags', '**', 'semantic', '*', '*', 'run.json'), recursive=True)
    rows = []
    for path in sorted(paths):
        with open(path) as f:run=json.load(f)
        if run.get('state')!='complete':continue
        folder = os.path.dirname(path)
        if not args.out:
            recording_dir = os.path.dirname(folder) if not args.root else semantic_directory(run['sources'][0]['path'])
            print(export_recording_map(folder, os.path.join(recording_dir, 'gallery')))
            continue
        export_final_map(folder, gallery=args.out)
        title=run.get('display_label_en') or run.get('run_id',os.path.basename(folder))
        name=gallery_image_path(run,folder).replace(os.sep, '/')
        recording=name.split('/', 1)[0]
        rows.append(dict(recording=recording,title=run.get('display_label',title),image=name,run=folder,
                         world=run['world_frame'],sources=[source['path'] for source in run.get('sources', [])]))
        print(name)
    if not args.out:
        return
    os.makedirs(args.out, exist_ok=True)
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

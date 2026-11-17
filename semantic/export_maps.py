#!/usr/bin/env python3
"""Export completed runs into one easy-to-find standalone PNG gallery."""
import argparse
import glob
import json
import os
import sys

ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)));sys.path.insert(0,ROOT)
from map_display import export_final_map,gallery_filename


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',default=os.path.join(ROOT,'results','semantic'))
    ap.add_argument('--out',default=os.path.join(ROOT,'results','semantic_maps'));args=ap.parse_args()
    os.makedirs(args.out,exist_ok=True);rows=[]
    for path in sorted(glob.glob(os.path.join(args.root,'*','run.json'))):
        with open(path) as f:run=json.load(f)
        if run.get('state')!='complete':continue
        folder=os.path.dirname(path);export_final_map(folder,gallery=args.out)
        title=run.get('display_label_en') or run.get('run_id',os.path.basename(folder))
        name=gallery_filename(run,folder)
        rows.append(dict(title=run.get('display_label',title),image=name,run=folder,world=run['world_frame']))
        print(name)
    with open(os.path.join(args.out,'index.json'),'w') as f:json.dump(rows,f,indent=2,ensure_ascii=False)
    with open(os.path.join(args.out,'README.md'),'w') as f:
        f.write('# 语义地图展示\n\n每张 PNG 为完整会话的 2400×1600 展示图；可脱离机器人、bag 和模型打开。编号与对应运行的阅读器一致，各版本编号不通用。点是可见表面候选，不是验证后的物体中心。\n\n')
        f.write('原始结果目录：`'+os.path.abspath(args.root)+'`\n\n')
        for row in rows:f.write('- [%s](%s) — %s；来源 `%s`\n'%(row['title'],row['image'],row['world'],row['run']))
    print('Gallery: '+os.path.abspath(args.out))


if __name__=='__main__':main()

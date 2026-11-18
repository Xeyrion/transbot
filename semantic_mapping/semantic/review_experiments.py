#!/usr/bin/env python3
"""Render saved comparisons; uses no model and performs no source-bag writes."""
import argparse
import hashlib
import json
import math
import os
import sys

HERE=os.path.dirname(os.path.abspath(__file__))
ROOT=os.path.dirname(HERE);sys.path.insert(0,ROOT)
from semantic_replay import SemanticReplay,draw_detections,replay_geometry


def main():
    ap=argparse.ArgumentParser();ap.add_argument('comparison');args=ap.parse_args()
    path=os.path.abspath(args.comparison);parent=os.path.dirname(path)
    data=json.load(open(path));runs={r['code']:r for r in data['variants']}
    from PIL import Image,ImageDraw
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.transforms import Affine2D
    import numpy as np
    import yaml
    bag=json.load(open(runs['B0']['path']+'/run.json'))['sources'][0]['path']
    sheet=Image.new('RGB',(1280,1040),(238,241,244));draw=ImageDraw.Draw(sheet)
    for row,frame_id in enumerate(('0059','0091')):
        meta=json.load(open(runs['B0']['path']+'/'+frame_id+'/meta.json'))
        source_bag=os.path.join(os.path.dirname(bag),meta['bag'])
        for column,code in enumerate(('B0','B1')):
            replay=SemanticReplay(runs[code]['path'],source_bag)
            sample=replay.nearest(meta['rgb_record']);assert sample['id']==frame_id
            dets=replay.payload(sample)['detections']
            with Image.open(sample['folder']+'/rgb.png') as image:
                annotated=draw_detections(image,dets)
            top=row*520;left=column*640
            sheet.paste(annotated,(left,top+40))
            accepted=sum(d['projection']=='accepted' for d in dets if d['label']=='chair')
            title='%s %s | sample %s | accepted chair %d'%(code,'centre ROI' if code=='B0' else 'mask surface',frame_id,accepted)
            draw.text((left+8,top+8),title,fill=(25,35,45))
            draw.text((left+8,top+23),'Identical RGB, detections and masks; projection differs.',fill=(80,90,100))
    sheet.save(os.path.join(parent,'depth_preview.png'))
    fig,axes=plt.subplots(2,4,figsize=(16,9))
    models={c:json.load(open(runs[c]['path']+'/objects.json')) for c in runs}
    reference=json.load(open(runs['A0']['path']+'/run.json'));linked=replay_geometry(reference,runs['A0']['path'])
    info=yaml.safe_load(open(linked['yaml']));image_path=info['image']
    if not os.path.isabs(image_path):image_path=os.path.join(os.path.dirname(linked['yaml']),image_path)
    raster=np.asarray(Image.open(image_path));h,w=raster.shape;res=info['resolution'];ox,oy,yaw=info['origin']
    objects=[o for model in models.values() for o in model['objects']]
    xs=[o['x'] for o in objects];ys=[o['y'] for o in objects]
    limits=(min(xs)-1,max(xs)+1,min(ys)-1,max(ys)+1)
    colors={'chair':'#0077bb','laptop':'#ee7733','tv':'#aa4499'}
    for ax,code in zip(axes.flat,sorted(runs)):
        transform=Affine2D().rotate(yaw).translate(ox,oy)+ax.transData
        ax.imshow(raster,cmap='gray',vmin=0,vmax=255,origin='upper',extent=(0,w*res,0,h*res),transform=transform)
        for label in colors:
            pts=[o for o in models[code]['objects'] if o['label']==label]
            if pts:ax.scatter([o['x'] for o in pts],[o['y'] for o in pts],s=18,c=colors[label],label=label,edgecolors='white',linewidths=.35)
        ax.set_xlim(limits[:2]);ax.set_ylim(limits[2:]);ax.set_aspect('equal');ax.grid(alpha=.2)
        counts=runs[code]['statistics']['candidates']
        ax.set_title('%s: %s'%(code,', '.join('%s %d'%(k,v) for k,v in counts.items())),fontsize=10)
        ax.tick_params(labelsize=7)
    fig.suptitle('Same map and axes | full-session surface candidates, NOT ground truth counts',fontsize=13)
    axes.flat[0].legend(fontsize=7,loc='lower left');fig.tight_layout(rect=(0,0,1,.95))
    fig.savefig(os.path.join(parent,'fusion_preview.png'),dpi=130);plt.close(fig)
    # Exact controls: changing a projector or fuser cannot change input detections.
    checks={}
    for group in (('A0','A1'),('B0','B1','B2'),('C0','C1','C2')):
        fingerprints=[];detections=[]
        for code in group:
            run=json.load(open(runs[code]['path']+'/run.json'));fingerprints.append(run['sample_fingerprint'])
            obs=json.load(open(runs[code]['path']+'/observations.json'))
            value=[{k:d[k] for k in ('source_frame','label','score','bbox_xywh')} for d in obs['detections']]
            detections.append(hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest())
        checks['/'.join(group)]=dict(same_samples=len(set(fingerprints))==1,same_detections=len(set(detections))==1)
        assert checks['/'.join(group)]==dict(same_samples=True,same_detections=True)
    with open(os.path.join(parent,'controls.json'),'w') as f:json.dump(checks,f,indent=2)
    lines=['# 深度与融合对照','',
           '在数据阅读中打开 session_20261117_071239 的 bag，在“语义回放”选择 A0–C2。编号是运行内编号，不跨版本对应同一物体。',
           '', '| 版本 | 方案 | RGB-D 帧数 | 椅子检测 | 椅子有效投影 | 椅子候选 |',
           '|---|---|---:|---:|---:|---:|']
    for code in sorted(runs):
        row=runs[code];s=row['statistics']
        lines.append('| %s | %s | %d | %d | %d | %d |'%(code,row['label'],s['frames'],s['detections'].get('chair',0),s['accepted'].get('chair',0),s['candidates'].get('chair',0)))
    lines+=['','B0/B1 使用完全相同的模型输出、RGB、深度和地图，仅改变深度取值；A0/A1 及 C0/C1/C2 的输入也经检查。A0 与原 YOLO26s 检测基线不同于分割模型 B0；不能跨模型直接把变化归因于投影。',
            '', '1Hz 与 3Hz 使用同一包、相同时间配对与几何约束，但采样图像不完全相同。3Hz 实际得到 397 组，未补造超出 RGB-D 配对门限的帧。',
            '', 'C2 是启发式实验：同帧低重叠检测框不合并；椅子使用水平 radius=0.45m、垂直尺度 0.8m 的椭球约束。C1/C2 另要求至少 1 秒跨度，且平移达到 0.15m 或视角达到 8°。这不是完整目标跟踪。',
            '', '两个示例图中的掩膜表示模型输出；B0 仍用中心 ROI 取深度，B1 才使用掩膜。橙色提示区分有效深度不足、混杂、支持不足和歧义。',
            '', '这些统计没有人工物体真值，投影更多不等于更准确，候选点更多也不等于识别到更多真实物体。仍需检查重复点、误合并、错误表面与位置。',
            '', '独立短包 session_20261117_060548_worklab_11_16 的 D0/D1 为相同 36 组 2Hz RGB-D：152 次椅子检测，中心方案有效 136 次、掩膜方案 148 次；候选 5/4。该包仅 19.55 秒且录制提前终止，使用 odom，不作为完整占用地图或精度验证。',
            '', '实验数据用硬链接共享，只复制失败时回退；共享样本不得原地修改。原始 bag、校准配置和当前地图未被覆盖。失败的早期依赖测试保存在 archive/semantic_experiments/。']
    with open(os.path.join(parent,'README.md'),'w') as f:f.write('\n'.join(lines)+'\n')
    print('Rendered '+parent)


if __name__=='__main__':main()

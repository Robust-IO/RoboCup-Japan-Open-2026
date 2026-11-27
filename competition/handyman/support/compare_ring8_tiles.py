"""Uniform sliding-window replay; raw candidates, not merged accuracy."""
import base64
import hashlib
import html
import json
from pathlib import Path
import numpy as np
from PIL import Image
from ultralytics import YOLO

ROOT=Path('/__HANDYMAN_PACKAGE__/runtime')
OUT=ROOT/'pose6-ring8-tiles-20261005-v1'

def main():
    reference=json.loads((ROOT/'pose6-ring8-comparison-20261005-v1/predictions.json').read_text())
    weights=ROOT/'handyman30-pose6-ft-v1/weights/best.pt'
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    if sha(weights)!=reference['weights']['new']['sha256']:raise ValueError('Weights changed')
    OUT.mkdir(exist_ok=False);model=YOLO(str(weights));rows=[];images={}
    for view in range(8):
        source=Path('/mnt/d/search-ring-review-20260919')/f'view{view}.png'
        old=[r for r in reference['rows'] if r['view']==view and r['model']=='new']
        if any(r['sha256']!=sha(source) for r in old):raise ValueError('Image changed')
        images[view]='data:image/png;base64,'+base64.b64encode(source.read_bytes()).decode()
        for r in old:rows.append(dict(view=view,mode=f"full-{r['imgsz']}",detections=r['detections']))
        rgb=np.asarray(Image.open(source).convert('RGB'))
        if rgb.shape!=(480,640,3):raise ValueError('Unexpected image shape')
        for size in (640,1280):
            ds=[]
            for y in (0,120,240):
                for x in (0,160,320):
                    # Ultralytics numpy input is BGR, unlike PIL RGB.
                    crop=rgb[y:y+240,x:x+320,::-1].copy()
                    p=model.predict(crop,imgsz=size,conf=.1,iou=.7,device=0,verbose=False)[0]
                    for b in p.boxes:
                        box=b.xyxy[0].tolist();global_box=[box[0]+x,box[1]+y,box[2]+x,box[3]+y]
                        ds.append(dict(name=p.names[int(b.cls.item())],confidence=float(b.conf.item()),xyxy=global_box,
                            tile=[x,y,320,240],touches_tile_edge=box[0]<2 or box[1]<2 or box[2]>318 or box[3]>238))
            rows.append(dict(view=view,mode=f'tiles-{size}',detections=ds))
        print('view',view,'complete',flush=True)
    report=dict(weights_sha256=sha(weights),conf=.1,tile_size=[320,240],stride=[160,120],rows=rows,
        limitations=['Overlapping raw tile outputs contain duplicates; not unique object counts.',
        'Enlargement adds no captured detail; no complete ground truth or accuracy claim.',
        'User confirmed vase is not rabbit and floor planter is not pink cup; compare spatial boxes.',
        'No robot, training, model deployment, room gate or self filter changes.'])
    (OUT/'predictions.json').write_text(json.dumps(report,indent=2))
    page=['<!doctype html><meta charset="utf-8"><title>八方向整图与分块对比</title><style>body{font:16px sans-serif;margin:20px}section{display:grid;grid-template-columns:1fr 1fr;gap:12px}svg{width:100%}article{border:1px solid #aaa;padding:8px}li{font-size:13px}</style><h1>八方向整图与分块对比 · 同一新模型</h1><p>全图640/1280；固定320×240分块，50%重叠，分别输入640/1280。阈值0.1。分块结果未合并，重复框不代表多个物品，不计算准确率。</p>']
    for v in range(8):
        page.append(f'<h2>方向 {v+1} / view{v}</h2><section>')
        for r in [r for r in rows if r['view']==v]:
            page.append(f'<article><h3>{r["mode"]}</h3><svg viewBox="0 0 640 480"><image href="{images[v]}" width="640" height="480"/>')
            for d in r['detections']:
                x,y,x2,y2=d['xyxy'];label=html.escape(f"{d['name']} {d['confidence']:.2f}")
                page.append(f'<rect x="{x}" y="{y}" width="{x2-x}" height="{y2-y}" fill="none" stroke="lime" stroke-width="1"><title>{label}</title></rect><text x="{x}" y="{max(10,y-2)}" font-size="9" fill="red">{label}</text>')
            page.append('</svg><ul>'+''.join(f'<li>{html.escape(d["name"])} {d["confidence"]:.3f} @ {[round(z) for z in d["xyxy"]]}</li>' for d in r['detections'])+'</ul></article>')
        page.append('</section>')
    (OUT/'index.html').write_text('\n'.join(page),encoding='utf-8')
    print(str(OUT),flush=True)

if __name__=='__main__':main()

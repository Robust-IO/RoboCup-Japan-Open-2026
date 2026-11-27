"""Replay archived eight-direction main observation images; no robot control."""
import hashlib
import html
import json
from pathlib import Path
from ultralytics import YOLO

def main():
    root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
    source=Path('/mnt/d/search-ring-review-20260919')
    out=root/'pose6-ring8-comparison-20261005-v1'
    out.mkdir(exist_ok=False)
    weights={'old':root/'handyman30-salt-pepper-scene-ft-v1/weights/best.pt', 'new':root/'handyman30-pose6-ft-v1/weights/best.pt'}
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert sha(weights['old'])=='3d822e2fa3f13a2a237ca3be8ecae91f929991a279a139de2d69dd99964f594f'
    rows=[]
    for version,path in weights.items():
        model=YOLO(str(path))
        for size in (640,1280):
            for index in range(8):
                image=source/f'view{index}.png'
                pred=model.predict(str(image),imgsz=size,conf=.1,iou=.7,device=0,verbose=False)[0]
                name=f'view{index}-{version}-{size}.jpg'
                pred.save(filename=str(out/name))
                rows.append(dict(view=index,model=version,imgsz=size,source=str(image),sha256=sha(image),annotated=name,
                    detections=[dict(name=pred.names[int(b.cls.item())],confidence=float(b.conf.item()),xyxy=b.xyxy[0].tolist()) for b in pred.boxes]))
            print(version,size,'8/8',flush=True)
    report=dict(weights={k:dict(path=str(v),sha256=sha(v)) for k,v in weights.items()},conf=.1,iou=.7,rows=rows,
                limitations=['Archived replay, no new Unity images.','No complete human box labels: not accuracy or recall.','Detections do not establish room membership.'])
    (out/'predictions.json').write_text(json.dumps(report,indent=2))
    page=['<!doctype html><meta charset="utf-8"><title>主观察点八图对比</title><style>body{font:16px sans-serif;margin:24px;background:#eee}section{display:grid;grid-template-columns:1fr 1fr;gap:16px}img{width:100%}article{background:white;padding:12px}h2{margin-top:35px}</style><h1>主观察点八方向：旧模型 / 姿态微调模型</h1><p>历史原图回放；置信度门槛 0.1。无完整人工框标注，不计算准确率；房外物品不等于房内目标。1280 为原图放大输入。</p>']
    for size in (640,1280):
        for index in range(8):
            page.append(f'<h2>输入 {size} · 方向 {index+1}（view{index}）</h2><section>')
            for version in ('old','new'):
                r=next(r for r in rows if (r['view'],r['model'],r['imgsz'])==(index,version,size))
                text='；'.join(f"{d['name']} {d['confidence']:.1%}" for d in r['detections']) or '无检测'
                page.append(f'<article><h3>{"旧模型" if version=="old" else "新模型"}</h3><img src="{r["annotated"]}"><p>{html.escape(text)}</p></article>')
            page.append('</section>')
    (out/'index.html').write_text('\n'.join(page),encoding='utf-8')
    print(json.dumps(report),flush=True)

if __name__=='__main__':main()

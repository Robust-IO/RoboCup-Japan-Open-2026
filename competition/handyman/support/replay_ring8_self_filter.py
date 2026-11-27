"""Frame-bound gripper-region diagnostic, deliberately NOT a runtime filter.

The provisional rectangle is a conservative experiment on eight archived frames,
not a calibrated silhouette. Preserve all detections; never infer object absence.
"""
import argparse
import hashlib
import html
import json
from pathlib import Path
import math

ROI = (185, 466, 305, 480)

def classify(box, *, verified_frame=False, phase='search'):
    if not verified_frame or phase != 'search':
        return 'keep_unverified_context'
    if len(box) != 4 or not all(math.isfinite(v) for v in box):
        return 'keep_invalid_box'
    x1,y1,x2,y2=box
    if not 0<=x1<x2<=640 or not 0<=y1<y2<=480:
        return 'keep_invalid_box'
    a,b,c,d=ROI
    overlap=max(0,min(x2,c)-max(x1,a))*max(0,min(y2,d)-max(y1,b))
    ratio=overlap/((x2-x1)*(y2-y1))
    return 'flag_self_region' if ratio>=.95 else 'keep'

def run(source, output):
    report=json.loads(source.read_text())
    rows=report['rows']
    if len(rows)!=32 or {(r['view'],r['model'],r['imgsz']) for r in rows}!={(v,m,s) for v in range(8) for m in ['old','new'] for s in [640,1280]}:
        raise ValueError('Unexpected replay set')
    from PIL import Image,ImageDraw
    for r in rows:
        p=Path(r['source'])
        if p.name!=f"view{r['view']}.png" or p.parent!=Path('/mnt/d/search-ring-review-20260919'):
            raise ValueError('Not the reviewed archive')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=r['sha256']:
            raise ValueError('Source changed')
        with Image.open(p) as im:
            if im.size!=(640,480): raise ValueError('Unexpected image size')
    output.mkdir(exist_ok=False)
    totals={}
    page=['<!doctype html><meta charset="utf-8"><title>夹爪区域离线复核</title><style>body{font:16px sans-serif;margin:24px}img{max-width:640px;width:100%}section{display:flex;flex-wrap:wrap;gap:20px}article{max-width:640px}</style><h1>夹爪区域离线复核</h1><p>仅绑定八张历史图，不是在线过滤配置。紫框为暂定区域；橙框为疑似自身检测，绿框保留。原预测不删除。区域尚需精确校准，过滤不代表目标不存在。</p>']
    for r in rows:
        with Image.open(r['source']) as im: canvas=im.convert('RGB')
        draw=ImageDraw.Draw(canvas)
        draw.rectangle(ROI,outline='magenta',width=2)
        counts=totals.setdefault(f"{r['model']}-{r['imgsz']}",dict(total=0,flagged=0,kept=0))
        lines=[]
        for det in r['detections']:
            decision=classify(det['xyxy'],verified_frame=True)
            det['self_region_review']=decision
            flag=decision=='flag_self_region'
            counts['total']+=1;counts['flagged' if flag else 'kept']+=1
            draw.rectangle(det['xyxy'],outline='orange' if flag else 'green',width=2)
            label=f"{det['name']} {det['confidence']:.2f} {'SELF?' if flag else 'KEEP'}"
            draw.text((det['xyxy'][0],max(0,det['xyxy'][1]-12)),label,fill='orange' if flag else 'green')
            lines.append(label)
        filename=f"view{r['view']}-{r['model']}-{r['imgsz']}.jpg"
        canvas.save(output/filename)
        page.append(f'<article><h2>view{r["view"]} / {r["model"]} / {r["imgsz"]}</h2><img src="{filename}"><p>{html.escape("; ".join(lines))}</p></article>')
    report.update(self_filter=dict(mode='offline_review_only',roi=ROI,minimum_box_overlap=.95,totals=totals,
        deployed=False,limitations=['Provisional rectangle, not exact gripper silhouette.','A real object fully inside the region could be flagged; no runtime suppression authorized by this result.','Moving head/arm and grasped objects require different handling.']))
    (output/'review.json').write_text(json.dumps(report,indent=2))
    (output/'index.html').write_text('\n'.join(page),encoding='utf-8')
    print(json.dumps(report['self_filter'],indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();run(args.source,args.output)

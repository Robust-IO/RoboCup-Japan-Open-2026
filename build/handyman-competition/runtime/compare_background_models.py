"""Whole-image background regression; no training, filtering or deployment."""
import hashlib
import html
import json
from pathlib import Path
from ultralytics import YOLO

ROOT = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def iou(a,b):
    inter=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union else 0


def main():
    out=ROOT/'background-live-comparison-20261007-v1'
    if out.exists(): raise ValueError('Output exists')
    archive=json.loads((ROOT/'live30-pose6-recheck-20261005-v1/report.json').read_text())
    review=json.loads((ROOT/'background-errors-review-v1/resolved-review-20261007.json').read_text())
    cases=[dict(id=c['id'],source=c['source'],sha256=c['sha256']) for c in archive['cases']]
    for i in range(8):
        p=Path('/mnt/d/search-ring-review-20260919')/f'view{i}.png'
        cases.append(dict(id=f'view{i}',source=str(p),sha256=sha(p)))
    weights={'old':ROOT/'handyman30-pose6-ft-v1/weights/best.pt','new':ROOT/'handyman30-background-ft-v1/weights/best.pt'}
    if sha(weights['old'])!=archive['weights']['new']['sha256']: raise ValueError('Baseline changed')
    for c in cases:
        if sha(Path(c['source']))!=c['sha256']: raise ValueError('Image changed')
    out.mkdir()
    models={k:YOLO(str(p)) for k,p in weights.items()}
    rows=[]
    for c in cases:
        for size in (640,1280):
            for version,model in models.items():
                r=model.predict(c['source'],imgsz=size,conf=.1,iou=.7,device=0,verbose=False)[0]
                name=f'{c["id"]}-{version}-{size}.jpg'
                r.save(filename=str(out/name))
                rows.append(dict(case=c['id'],imgsz=size,model=version,annotated=name,
                    detections=[dict(name=r.names[int(b.cls.item())],confidence=float(b.conf.item()),xyxy=b.xyxy[0].tolist()) for b in r.boxes]))
        print(c['id'],flush=True)
    changes=[]
    for h in review['rows']:
        row=next(r for r in rows if r['case']==h['case'] and r['imgsz']==h['imgsz'] and r['model']=='new')
        matches=[d for d in row['detections'] if iou(d['xyxy'],h['detection']['xyxy'])>=.5]
        same=[d for d in matches if d['name']==h['detection']['name']]
        changes.append(dict(review_id=h['id'],decision=h['human_decision'],note=h['human_note'],
            old=h['detection'],new_same_class_max=max((d['confidence'] for d in same),default=0),new_overlapping=matches))
    report=dict(cases=cases,weights={k:dict(path=str(p),sha256=sha(p)) for k,p in weights.items()},rows=rows,review_changes=changes,
        limitations=['Raw whole-frame conf=0.1, no additional NMS or tiles.', 'Human-reviewed old boxes are anchors, not complete ground truth.', 'IoU>=0.5 matching can miss shifted boxes; absence is not proof of solved false positives.', 'Counts include repeated images/scales/boxes.'])
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    page=['<!doctype html><meta charset="utf-8"><title>背景微调前后对比</title><style>body{font:16px sans-serif;margin:24px}section{display:flex;gap:16px}article{width:48%}img{width:100%}</style><h1>姿态模型 → 背景微调模型</h1><p>76张历史图片，640/1280整图输入，阈值0.1，无额外去重。旧框局部匹配不是完整准确率；请同时检查漏检与新误检。</p>']
    for c in cases:
        for size in (640,1280):
            page.append(f'<h2>{c["id"]} / {size}</h2><section>')
            for version in ('old','new'):
                r=next(r for r in rows if (r['case'],r['imgsz'],r['model'])==(c['id'],size,version))
                desc='；'.join(f'{d["name"]} {d["confidence"]:.1%}' for d in r['detections']) or '无检测'
                page.append(f'<article><h3>{version}</h3><img loading="lazy" src="{r["annotated"]}"><p>{html.escape(desc)}</p></article>')
            page.append('</section>')
    (out/'index.html').write_text('\n'.join(page),encoding='utf-8')
    for decision in ('background','correct'):
        group=[x for x in changes if x['decision']==decision]
        print(json.dumps(dict(decision=decision,anchors=len(group),same_class_remaining=sum(x['new_same_class_max']>=.1 for x in group),any_class_overlap=sum(bool(x['new_overlapping']) for x in group))))
    print('OUTPUT',out,flush=True)


if __name__=='__main__': main()

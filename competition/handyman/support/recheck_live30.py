"""Historical whole-frame replay and per-case visual comparison, no training."""
import base64
import hashlib
import html
import json
from collections import defaultdict
from pathlib import Path
from ultralytics import YOLO

ROOT=Path('/__HANDYMAN_PACKAGE__/runtime')
OUT=ROOT/'live30-pose6-recheck-20261005-v1'
ALIASES={'apple':'apple','bear':'bear_doll','camera':'camera','canned-juice':'canned_juice',
 'plastic-pair':['empty_plastic_bottle','filled_plastic_bottle'],'duck-dog':['toy_duck','dog_doll'],
 'capture':['salt','ground_pepper']}

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    reference=json.loads((ROOT/'pose6-ring8-comparison-20261005-v1/predictions.json').read_text())
    weights={k:Path(v['path']) for k,v in reference['weights'].items()}
    for k,p in weights.items():
        if sha(p)!=reference['weights'][k]['sha256']:raise ValueError('Model changed')
    pool={};unavailable=[]
    for group in ['live30-20261003','live30-20261004','salt-pepper-live-20261003']:
        for file in sorted((ROOT/group).rglob('predictions.json')):
            rows=json.loads(file.read_text())
            if not isinstance(rows,list):continue
            for row in rows:
                source=row.get('source')
                if not source:continue
                p=Path(source)
                if not p.is_absolute():p=file.parent/p
                if not p.is_file():
                    entry=dict(evidence=str(file),source=source)
                    if entry not in unavailable:unavailable.append(entry)
                    continue
                source=str(p)
                if row.get('sha256') and sha(p)!=row['sha256']:raise ValueError('Original changed '+source)
                c=pool.setdefault(source,dict(source=source,sha256=sha(p),evidence=[],targets=[],target_basis=[]))
                if str(file) not in c['evidence']:c['evidence'].append(str(file))
                target=row.get('user_target')
                if isinstance(target,str) and target:
                    if target not in c['targets']:c['targets'].append(target)
                    if 'historical_user_target' not in c['target_basis']:c['target_basis'].append('historical_user_target')
                elif not c['targets']:
                    for prefix,labels in ALIASES.items():
                        if file.parent.name.startswith(prefix):
                            c['targets']=labels if isinstance(labels,list) else [labels]
                            c['target_basis']=['comparison_directory_inference_requires_review'];break
    OUT.mkdir(exist_ok=False);cases=list(pool.values())
    models={k:YOLO(str(p)) for k,p in weights.items()};names=list(models['new'].names.values())
    for i,c in enumerate(cases):
        c['id']=f'case-{i+1:03d}';c['results']=[]
        for version,model in models.items():
            for size in [640,1280]:
                p=model.predict(c['source'],imgsz=size,conf=.1,iou=.7,device=0,verbose=False)[0]
                ds=[dict(name=p.names[int(b.cls.item())],confidence=float(b.conf.item()),xyxy=b.xyxy[0].tolist()) for b in p.boxes]
                c['results'].append(dict(model=version,imgsz=size,detections=ds))
        print(c['id'],','.join(c['targets']) or 'unlabeled/negative',flush=True)
    coverage={name:[c['id'] for c in cases if name in c['targets']] for name in names}
    report=dict(cases=cases,coverage=coverage,unavailable_sources=unavailable,weights=reference['weights'],conf=.1,
        limitations=['Historical targets are image-level, not complete box annotations.',
        'Directory-inferred labels explicitly marked; not confirmed ground truth.',
        'No accuracy claim from class-name presence. Background objects may be real or false positives.',
        'Whole frames only. No tiles, room gating, self filtering, training or deployment.'])
    (OUT/'report.json').write_text(json.dumps(report,indent=2))
    page=['<!doctype html><meta charset="utf-8"><title>30类历史实拍逐图复核</title><style>body{font:16px sans-serif;margin:22px;background:#f3f5f7}section{display:grid;grid-template-columns:1fr 1fr;gap:12px}article{background:white;padding:10px}svg{width:100%}details{margin:15px 0}li{font-size:13px}</style><h1>30类历史实拍逐图复核</h1><p>同图比较旧模型与姿态微调模型，整图640/1280，阈值0.1。类别是否出现不等于框正确；不能将背景预测直接当作正确识别。图片级目标来源推断已标明。</p><h2>类别索引</h2>']
    for name,ids in coverage.items():
        page.append('<p>'+name+': '+(' · '.join(f'<a href="#{i}">{i}</a>' for i in ids) or '缺少可确定目标记录')+'</p>')
    for c in cases:
        raw=base64.b64encode(Path(c['source']).read_bytes()).decode()
        from PIL import Image
        with Image.open(c['source']) as im:w,h=im.size
        page.append(f'<details id="{c["id"]}" open><summary>{c["id"]} | {html.escape(", ".join(c["targets"]) or "无明确目标／负样本")} | {html.escape(Path(c["source"]).parent.name+"/"+Path(c["source"]).name)}</summary><p>依据：{html.escape(",".join(c["target_basis"]))}</p><section>')
        for size in [640,1280]:
            for version in ['old','new']:
                r=next(r for r in c['results'] if r['model']==version and r['imgsz']==size)
                page.append(f'<article><h3>{version} · {size}</h3><svg viewBox="0 0 {w} {h}"><image href="data:image/png;base64,{raw}" width="{w}" height="{h}"/>')
                for d in r['detections']:
                    x,y,x2,y2=d['xyxy'];label=html.escape(f"{d['name']} {d['confidence']:.2f}")
                    page.append(f'<rect x="{x}" y="{y}" width="{x2-x}" height="{y2-y}" fill="none" stroke="lime"/><text x="{x}" y="{max(10,y-2)}" font-size="10" fill="red">{label}</text>')
                page.append('</svg><ul>'+''.join(f'<li>{html.escape(d["name"])}: {d["confidence"]:.3f}</li>' for d in r['detections'])+'</ul></article>')
        page.append('</section></details>')
    (OUT/'index.html').write_text('\n'.join(page),encoding='utf-8')
    print(json.dumps(dict(output=str(OUT),frames=len(cases),unavailable_sources=unavailable,coverage={k:len(v) for k,v in coverage.items()},missing=[k for k,v in coverage.items() if not v])),flush=True)

if __name__=='__main__':main()

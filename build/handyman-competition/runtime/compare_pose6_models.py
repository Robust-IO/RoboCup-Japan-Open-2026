"""Paired diagnostic and historical camera replay; no deployment or GT invention."""
import hashlib
import json
from pathlib import Path
from collections import defaultdict
from ultralytics import YOLO

ROOT = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
OUT = ROOT/'pose6-comparison-20261005-v1'

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def iou(a,b):
    intersection=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union else 0

def main():
    OUT.mkdir(exist_ok=False)
    cases=[]
    for line in (ROOT/'pose6-baseline-20261005-v1/predictions.jsonl').read_text().splitlines():
        r=json.loads(line)
        if r['imgsz']==640:
            assert sha(Path(r['source']))==r['source_sha256']
            cases.append(dict(kind='pose',source=r['source'],target=r['target'],box=r['geometry_box'],pose=r['pose'],index=r['index']))
    seen=set()
    for group in ['live30-20261003','live30-20261004','salt-pepper-live-20261003']:
        for p in sorted((ROOT/group).rglob('predictions.json')):
            rows=json.loads(p.read_text())
            if not isinstance(rows,list): continue
            for r in rows:
                source=r.get('source')
                if not source or source in seen or not Path(source).is_file(): continue
                seen.add(source)
                cases.append(dict(kind='live',source=source,target=r.get('user_target'),evidence=str(p)))
    models={'old':ROOT/'handyman30-salt-pepper-scene-ft-v1/weights/best.pt','new':ROOT/'handyman30-pose6-ft-v1/weights/best.pt'}
    assert sha(models['old'])=='3d822e2fa3f13a2a237ca3be8ecae91f929991a279a139de2d69dd99964f594f'
    (OUT/'manifest.json').write_text(json.dumps(dict(cases=cases,weights={k:dict(path=str(v),sha256=sha(v)) for k,v in models.items()},conf=.1,iou=.7),indent=2))
    results=[]
    for version,weights in models.items():
        model=YOLO(str(weights))
        for size in [640,1280]:
            for n,c in enumerate(cases):
                p=model.predict(c['source'],imgsz=size,conf=.1,iou=.7,device=0,verbose=False)[0]
                ds=[dict(name=p.names[int(b.cls.item())],confidence=float(b.conf.item()),xyxy=b.xyxy[0].tolist()) for b in p.boxes]
                row=dict(**c,model=version,imgsz=size,detections=ds)
                if c['kind']=='pose':
                    row['matched']=any(d['name']==c['target'] and iou(d['xyxy'],c['box'])>=.5 for d in ds)
                    row['wrong']=any(d['name']!=c['target'] and iou(d['xyxy'],c['box'])>=.3 for d in ds)
                results.append(row)
                with (OUT/'predictions.jsonl').open('a') as f: f.write(json.dumps(row)+'\n')
            print(version,size,'completed',len(cases),flush=True)
    groups=defaultdict(list)
    for r in results:
        if r['kind']=='pose': groups[(r['model'],r['imgsz'],r['target'])].append(r)
    summary=[dict(model=k[0],imgsz=k[1],target=k[2],frames=len(rs),matched=sum(r['matched'] for r in rs),wrong=sum(r['wrong'] for r in rs)) for k,rs in groups.items()]
    report=dict(pose=summary,live_unique_images=len(seen),limitations=['Pose diagnostic informed training design: not independent validation.','Live replay lacks reviewed boxes; predictions and target-name presence are not accuracy.','1280 upsamples existing images.'])
    (OUT/'summary.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)

if __name__=='__main__': main()

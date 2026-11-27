"""Offline screen of existing RGBD captures; does not modify old data or rerun YOLO."""
import argparse,json
from collections import Counter
from pathlib import Path
import numpy as np
from PIL import Image
from view_health import assess_view

def check(folder):
    meta=json.loads((folder/'request.json').read_text()); d=meta['depth']
    if d['encoding'] not in ('16UC1','32FC1'): raise ValueError('unsupported encoding')
    dtype=np.dtype(('>' if d['is_bigendian'] else '<')+('u2' if d['encoding']=='16UC1' else 'f4'))
    data=(folder/'depth.bin').read_bytes()
    if len(data)!=d['step']*d['height'] or d['step']<d['width']*dtype.itemsize: raise ValueError('bad stride/buffer')
    depth=np.ndarray((d['height'],d['width']),dtype=dtype,buffer=data,strides=(d['step'],dtype.itemsize)).astype(float)
    if d['encoding']=='16UC1': depth*=.001
    rgb=np.asarray(Image.open(folder/'rgb.png').convert('RGB'))
    old=json.loads((folder/'result.json').read_text())
    if 'error' in old: raise ValueError('original inference failed')
    return dict(folder=str(folder),original_detections=[x['name'] for x in old['detections']],health=assess_view(rgb,depth))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--runs',type=Path,nargs='+',required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); rows=[]; errors=[]
    for run in a.runs:
        folders=sorted(x.parent for x in run.glob('[0-9]*/request.json'))
        if not folders: raise ValueError('No captures: '+str(run))
        for folder in folders:
            try: rows.append(check(folder))
            except (OSError,ValueError,KeyError) as exc: errors.append(dict(folder=str(folder),error=str(exc)))
    summary={}
    for name,predicate in [('target_detected',lambda r:'canned_juice' in r['original_detections']),('no_detections',lambda r:not r['original_detections'])]:
        group=[r for r in rows if predicate(r)]
        summary[name]=dict(count=len(group),reasons=dict(Counter(r['health']['reason'] for r in group)))
        for metric in ['valid_depth_fraction','rgb_spatial_std']:
            values=[r['health'][metric] for r in group if metric in r['health']]
            if values: summary[name][metric+'_range']=[min(values),max(values)]
    report=dict(offline=True,coverage_verified=False,summary=summary,errors=errors,samples=rows,
                caveat='Grouping uses old detector output, not independent ground-truth labels. Static repeated views do not establish generalisation.')
    with a.output.open('x') as f: json.dump(report,f,indent=2,allow_nan=False)
    print(json.dumps(dict(summary=summary,errors=errors,output=str(a.output)),indent=2))

if __name__=='__main__': main()

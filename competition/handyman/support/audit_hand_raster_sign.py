"""Independent image-feature / head-depth check of two fixed raster conventions."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from hand_camera_ray import rotation


def main():
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    s=json.loads((a.capture/'snapshot.json').read_text());d=np.load(a.capture/'depth_m.npy')
    head=cv2.imread(str(a.capture/'rgb.png'));hand=cv2.imread(str(a.capture/'hand.png'))
    sift=cv2.SIFT_create();kh,dh=sift.detectAndCompute(cv2.cvtColor(head,cv2.COLOR_BGR2GRAY),None);kw,dw=sift.detectAndCompute(cv2.cvtColor(hand,cv2.COLOR_BGR2GRAY),None)
    if dh is None or dw is None:raise ValueError('No descriptors')
    matcher=cv2.BFMatcher()
    def ratios(left,right):
        return {m.queryIdx:m.trainIdx for pair in matcher.knnMatch(left,right,k=2) if len(pair)==2 for m,n in [pair] if m.distance<.7*n.distance}
    f=ratios(dh,dw);b=ratios(dw,dh);pairs=[(i,j) for i,j in f.items() if b.get(j)==i]
    kr=np.array(s['rgb_k']).reshape(3,3);kwmat=np.array(s['hand_k']).reshape(3,3)
    rt=rotation(s['base_from_rgb']['quaternion']);tt=np.array(s['base_from_rgb']['translation'])
    rh=rotation(s['base_from_hand']['quaternion']);th=np.array(s['base_from_hand']['translation'])
    results=[]
    for i,j in pairs:
        u,v=kh[i].pt;x,y=int(round(u)),int(round(v))
        # Exclude central robot/apple region: test background, not the fitted target.
        if 180<u<380 and v>270:continue
        if not (1<=x<d.shape[1]-1 and 1<=y<d.shape[0]-1):continue
        z=d[y-1:y+2,x-1:x+2];good=z[np.isfinite(z)&(z>0)&(z<5)]
        if len(good)!=9 or np.ptp(good)>.03:continue
        base=rt@(np.linalg.solve(kr,[u,v,1])*np.median(good))+tt
        frame=rh.T@(base-th)
        row=dict(head_pixel=[u,v],hand_pixel=list(kw[j].pt),base_point=base.tolist())
        for name,m in [('legacy',[[0,-1,0],[-1,0,0],[0,0,1]]),('v_flipped',[[0,1,0],[-1,0,0],[0,0,1]])]:
            raster=np.array(m).T@frame
            if raster[2]<=0:raise ValueError('Behind hand camera')
            projected=kwmat@raster;uv=projected[:2]/projected[2]
            row[name]=dict(projected=uv.tolist(),error_px=float(np.linalg.norm(uv-kw[j].pt)))
        results.append(row)
    summary={}
    for name in ('legacy','v_flipped'):
        errors=[r[name]['error_px'] for r in results]
        summary[name]=dict(count=len(errors),median_error_px=float(np.median(errors)) if errors else None,under_5px=sum(e<5 for e in errors))
    report=dict(motion_authorized=False,method='SIFT mutual ratio 0.7; background-only, locally stable head depth; no fitted transform',
                summary=summary,matches=results,limitations=['Repeated wall patterns can produce false matches.',
                'Same capture pair, not a multi-pose calibration; head/hand stamps differ.'])
    with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()

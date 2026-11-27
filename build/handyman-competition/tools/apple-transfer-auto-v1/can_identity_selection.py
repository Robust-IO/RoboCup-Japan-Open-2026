"""Collapse repeated boxes of ONE can; never boost confidence or lower .8."""
import math


def overlap(a,b):
    x1=max(a[0],b[0]);y1=max(a[1],b[1]);x2=min(a[2],b[2]);y2=min(a[3],b[3])
    intersection=max(0.,x2-x1)*max(0.,y2-y1)
    return intersection/((a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection)


def confirmed_cans(detections):
    candidates=[]
    for value in detections:
        if value.get('name')!='canned_juice':continue
        box=value.get('box',[]);score=value.get('confidence')
        if (len(box)!=4 or type(score) not in (int,float) or not math.isfinite(score) or not .8<=score<=1.
                or not all(math.isfinite(v) for v in box) or box[2]<=box[0] or box[3]<=box[1]):continue
        candidates.append(value)
    kept=[]
    for value in sorted(candidates,key=lambda v:v['confidence'],reverse=True):
        if not any(overlap(value['box'],previous['box'])>=.85 for previous in kept):kept.append(value)
    return kept

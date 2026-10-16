"""Persistent GPU worker for the read-only RGBD node. No ROS imports."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from ultralytics import YOLO
from predict_rgb_readonly import merge_scales, validate_registry
from view_health import assess_view
from rgbd_worker_lifetime import worker_deadline

def atomic_json(path, value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,allow_nan=False))
    temporary.replace(path)

def measure(depth, mask, k):
    padded=np.pad(mask,2)
    core=np.ones_like(mask,dtype=bool)
    h,w=mask.shape
    for y in range(5):
        for x in range(5):
            core &= padded[y:y+h,x:x+w]
    valid=core & np.isfinite(depth) & (depth>0) & (depth<10)
    v,u=np.where(valid); z=depth[valid]
    result={'core_pixels':int(core.sum()),'valid_pixels':len(z)}
    if len(z):
        result.update(depth_p05_median_p95=np.percentile(z,[5,50,95]).tolist(),
                      optical_surface_median_m=np.median(np.column_stack(((u-k[2])*z/k[0],(v-k[5])*z/k[4],z)),axis=0).tolist())
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spool',type=Path,required=True)
    parser.add_argument('--weights',required=True)
    parser.add_argument('--deadline-monotonic',type=float)
    args=parser.parse_args()
    if args.deadline_monotonic is not None:
        worker_deadline(args.deadline_monotonic,time.monotonic())
    model=YOLO(args.weights); validate_registry(model.names)
    for size in [640,1280]:
        model.predict(np.zeros((480,640,3),np.uint8),imgsz=size,device=0,retina_masks=True,verbose=False)
    atomic_json(args.spool/'ready.json',{'ready':True})
    seen=set()
    deadline=worker_deadline(args.deadline_monotonic,time.monotonic())
    while time.monotonic()<deadline and not (args.spool/'stop').exists():
        for request in sorted(args.spool.glob('*/request.json')):
            if time.monotonic()>=deadline or (args.spool/'stop').exists(): break
            if request in seen: continue
            seen.add(request)
            try:
                meta=json.loads(request.read_text()); d=meta['depth']; k=meta['k']
                if d['encoding'] not in ('16UC1','32FC1'): raise ValueError('unsupported_depth_encoding')
                dtype=np.dtype(('>' if d['is_bigendian'] else '<')+('u2' if d['encoding']=='16UC1' else 'f4'))
                data=(request.parent/'depth.bin').read_bytes()
                if len(data)!=d['height']*d['step'] or d['step']<d['width']*dtype.itemsize: raise ValueError('invalid_depth_buffer')
                depth=np.ndarray((d['height'],d['width']),dtype=dtype,buffer=data,strides=(d['step'],dtype.itemsize)).astype(float)
                if d['encoding']=='16UC1': depth*=.001
                if len(k)!=9 or not all(np.isfinite(k)) or k[0]<=0 or k[4]<=0: raise ValueError('invalid_intrinsics')
                merged,_,conflicts=merge_scales([(size,model.predict(str(request.parent/'rgb.png'),imgsz=size,device=0,conf=.25,retina_masks=True,verbose=False)[0]) for size in [640,1280]])
                if tuple(merged.orig_shape)!=depth.shape: raise ValueError('shape_mismatch')
                detections=[]
                for i,box in enumerate(merged.boxes):
                    det={'name':model.names[int(box.cls.item())],'confidence':float(box.conf.item())}
                    det.update(measure(depth,merged.masks.data[i].cpu().numpy()>.5,k))
                    detections.append(det)
                merged.save(filename=str(request.parent/'detection.jpg'))
                from PIL import Image
                rgb=np.asarray(Image.open(request.parent/'rgb.png').convert('RGB'))
                atomic_json(request.parent/'result.json',{'detections':detections,'class_conflicts':conflicts,
                            'view_health':assess_view(rgb,depth)})
            except Exception as exc:
                atomic_json(request.parent/'result.json',{'error':str(exc)})
        time.sleep(.03)
    atomic_json(args.spool/'worker-exit.json',{'reason':'stop_requested' if (args.spool/'stop').exists() else 'deadline_reached',
                                            'deadline_monotonic':deadline,'monotonic_s':time.monotonic()})

if __name__=='__main__': main()

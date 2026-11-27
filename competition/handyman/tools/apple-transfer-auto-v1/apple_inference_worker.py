"""Persistent read-only YOLO worker. JSON-lines protocol, diagnostics on stderr."""
import argparse
import contextlib
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
from PIL import Image
from apple_rgbd_candidate import estimate
from apple_support_surface import check as check_support


def measured_proposal(rgb, depth, snapshot, detection):
    """Keep measured object geometry when the separate support estimate fails."""
    try:
        candidate=estimate(rgb,depth,snapshot,detection,geometry_proposal_only=True)
    except (ValueError,KeyError) as exc:
        return dict(detection=detection,rejected_reason=str(exc))
    try:
        support=check_support(depth,snapshot,detection,candidate)
    except (ValueError,KeyError) as exc:
        support=dict(support_consistent=False,reason=str(exc),
                     grasp_center_verified=False,motion_authorized=False)
    return dict(detection=detection,candidate=candidate,support=support)


def confirmed_geometry(proposals, detection):
    """Reuse this exact frame/detection's already fitted geometry; no re-fit."""
    if detection['name']!='apple' or not .8<=detection['confidence']<=1:
        raise ValueError('confirmed_full_frame_detection_required')
    matched=[p for p in proposals if p['detection'] is detection]
    if len(matched)!=1:raise ValueError('same_detection_geometry_required')
    proposal=matched[0]
    if 'rejected_reason' in proposal:raise ValueError(proposal['rejected_reason'])
    # This flag has the same meaning as estimate(...geometry_proposal_only=False):
    # full detector score passed its original gate. Identity/motion stay FALSE.
    return dict(proposal['candidate'],geometry_proposal_only=False),dict(proposal['support'])


def predict_full_frame(model,rgb,imgsz,device):
    """Existing pixels decoded ONCE; Ultralytics ndarray input is BGR."""
    return model.predict(rgb[:,:,::-1].copy(),imgsz=imgsz,conf=.1,device=device,verbose=False)[0]


def confirm_near_grasp(model,rgb,snapshot,proposals,detections,model_sha256,device):
    from apple_roi_confirmation import proposal_plan,assess
    start=time.monotonic();plan=proposal_plan(proposals,snapshot,detections);local=[]
    if plan['eligible']:
        x,y,right,bottom=plan['roi']
        # ndarray input is BGR; geometry uses untouched full-frame RGB/depth.
        crop=rgb[y:bottom,x:right,::-1].copy()
        with contextlib.redirect_stdout(sys.stderr):
            verification=model.predict(crop,imgsz=plan['input_size'],conf=.1,device=device,verbose=False)[0]
        local=[dict(name=model.names[int(c)],confidence=float(conf),box=[box[0]+x,box[1]+y,box[2]+x,box[3]+y])
               for box,c,conf in zip(verification.boxes.xyxy.cpu().tolist(),verification.boxes.cls.cpu().tolist(),verification.boxes.conf.cpu().tolist())]
    return dict(assess(plan,local),model_sha256=model_sha256,processing_s=time.monotonic()-start)


def main():
    p=argparse.ArgumentParser();p.add_argument('--registry',type=Path,required=True);p.add_argument('--imgsz',type=int,choices=[640,960,1280])
    p.add_argument('--confirm-apple-roi',action='store_true',help='One bounded same-frame near-grasp identity crop; no geometry replacement')
    a=p.parse_args()
    registry=json.loads(a.registry.read_text());weights=Path(registry['weights'])
    if a.imgsz:registry['imgsz']=a.imgsz
    digest=hashlib.sha256(weights.read_bytes()).hexdigest()
    if digest!=registry['sha256']:raise ValueError('model_hash_mismatch')
    with contextlib.redirect_stdout(sys.stderr):
        from ultralytics import YOLO
        model=YOLO(str(weights))
        if model.task!='detect' or len(model.names)!=30 or 'apple' not in model.names.values():raise ValueError('wrong_model')
        for _ in range(3):model.predict(np.zeros((480,640,3),dtype=np.uint8),imgsz=registry['imgsz'],conf=.1,device=registry['device'],verbose=False)
        if a.confirm_apple_roi:
            for _ in range(3):model.predict(np.zeros((320,320,3),dtype=np.uint8),imgsz=640,conf=.1,device=registry['device'],verbose=False)
    print(json.dumps(dict(status='ready',model_sha256=digest,imgsz=registry['imgsz'],near_grasp_roi_enabled=a.confirm_apple_roi)),flush=True)
    for line in sys.stdin:
        request=json.loads(line);start=time.monotonic();capture=Path(request['capture'])
        result=dict(request_id=request['request_id'],motion_authorized=False,imgsz=registry['imgsz'],model_sha256=digest)
        try:
            snapshot=json.loads((capture/'snapshot.json').read_text())
            rgb=np.asarray(Image.open(capture/'rgb.png').convert('RGB'));depth=np.load(capture/'depth_m.npy')
            read_complete=time.monotonic()
            with contextlib.redirect_stdout(sys.stderr):
                prediction=predict_full_frame(model,rgb,registry['imgsz'],registry['device'])
            detections=[dict(name=model.names[int(c)],confidence=float(conf),box=box) for box,c,conf in zip(prediction.boxes.xyxy.cpu().tolist(),prediction.boxes.cls.cpu().tolist(),prediction.boxes.conf.cpu().tolist())]
            apples=[d for d in detections if d['name']=='apple' and d['confidence']>=.8]
            result.update(detections=detections,depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'])
            detection_complete=time.monotonic()
            # Independent measured proposals, NOT confirmed targets. Original .8 gate below is unchanged.
            proposals=[]
            for detection in detections:
                if detection['name']!='apple':continue
                proposals.append(measured_proposal(rgb,depth,snapshot,detection))
            result['tracking_proposals']=proposals
            if a.confirm_apple_roi and request.get('confirm_apple_roi',True):
                result['near_grasp_roi_confirmation']=confirm_near_grasp(model,rgb,snapshot,proposals,detections,digest,registry['device'])
            if len(apples)!=1:raise ValueError('missing_or_ambiguous_confirmed_apple')
            candidate,support=confirmed_geometry(proposals,apples[0])
            result.update(status='candidate_only',candidate=candidate,support_evidence=support,
                processing_breakdown=dict(read_capture_s=read_complete-start,
                    full_detection_s=detection_complete-read_complete,
                    geometry_roi_and_result_s=time.monotonic()-detection_complete),
                duplicate_geometry_calculation_removed=True)
        except (ValueError,KeyError,OSError) as exc:result.update(status='rejected',reason=str(exc))
        result['worker_processing_s']=time.monotonic()-start
        print(json.dumps(result,allow_nan=False),flush=True)


if __name__=='__main__':main()

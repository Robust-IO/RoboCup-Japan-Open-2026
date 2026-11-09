"""Offline RGBD audit of the apple relative to a specific exported tabletop.

Low confidence proposals are geometry measurements, not new identity decisions.
This utility never authorizes or publishes movement.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
from apple_worker_client import Worker
from apple_rgbd_candidate import estimate
from hand_camera_ray import rotation


def inspect(capture,detections,dest,plane):
    s=json.loads((capture/'snapshot.json').read_text());rgb=np.asarray(Image.open(capture/'rgb.png').convert('RGB'));depth=np.load(capture/'depth_m.npy')
    tf=s['tracking_transforms']['map_from_base_at_depth'];r=rotation(tf['quaternion']);t=np.array(tf['translation']);bounds=dest['axis_aligned_bounds']
    center=np.array([bounds['center']['x'],bounds['center']['y']]);half=np.array([bounds['size']['x'],bounds['size']['y']])/2;coef=np.array(plane['plane_map'])
    apples=[];rejects=[]
    for det in detections:
        if det['name']!='apple':continue
        try:
            candidate=estimate(rgb,depth,s,det,geometry_proposal_only=True);p=np.array(candidate['center_m']);mapped=r@p+t;radius=candidate['radius_m'];height=float(np.r_[mapped[:2],1]@coef)
            apples.append(dict(confidence=det['confidence'],box=det['box'],candidate=candidate,center_map=mapped.tolist(),table_height_m=height,estimated_bottom_gap_m=float(mapped[2]-radius-height),footprint_margin_m=(half-np.abs(mapped[:2]-center)).tolist(),inside_table_with_apple_margin=bool((half-np.abs(mapped[:2]-center)>radius+.03).all())))
        except ValueError as exc:rejects.append(str(exc))
    return dict(capture=str(capture),apple_measurements=apples,rejections=rejects,source_stamp_ns=s['streams']['depth']['stamp_ns'],joints=s['joints'],motion_authorized=False)


def main():
    p=argparse.ArgumentParser();p.add_argument('captures',type=Path,nargs='+');p.add_argument('--registry',required=True);p.add_argument('--python',required=True);p.add_argument('--semantic-map',type=Path,required=True);p.add_argument('--destination-id',required=True);p.add_argument('--plane',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--inference-socket',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('output_exists')
    dest=next(x for x in json.loads(a.semantic_map.read_text())['destinations'] if x['id']==a.destination_id);plane=json.loads(a.plane.read_text())
    if a.inference_socket:
        from apple_shared_inference import SharedWorker
        worker=SharedWorker(a.inference_socket,a.registry,imgsz=960)
    else:
        worker=Worker(a.python,a.registry,a.output.with_suffix('.worker.log'),imgsz=960)
    results=[]
    try:
        for capture in a.captures:
            inference=worker.infer(capture);result=inspect(capture,inference.get('detections',[]),dest,plane);result['inference']=inference;results.append(result)
            print(json.dumps({k:v for k,v in result.items() if k!='inference'}),flush=True)
    finally:worker.close()
    a.output.write_text(json.dumps(dict(frames=results,motion_authorized=False),indent=2))


if __name__=='__main__':main()

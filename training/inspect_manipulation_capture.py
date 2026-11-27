"""Replay current or saved RGBD and detector outputs. No inference/motion publishers."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

from manipulation_profiles import object_profile
from object_surface_geometry import estimate


def inspect(capture,detections,name):
    capture=Path(capture);profile=object_profile(name)
    output=dict(status='observation_only',class_name=name,capture=str(capture),
                geometry_candidates=[],rejections=[],motion_authorized=False,physical_transfer_verified=False,
                identity_confirmed=False,does_not_exist_proven=False,
                note='This single-frame replay is not a fresh live identity or movement lease.')
    rgb=np.asarray(Image.open(capture/'rgb.png').convert('RGB'))
    depth=np.load(capture/'depth_m.npy',allow_pickle=False)
    snapshot=json.loads((capture/'snapshot.json').read_text())
    for item in detections:
        if item.get('name')!=name:continue
        detection=dict(item)
        if 'box' not in detection and 'xyxy' in detection:detection['box']=detection['xyxy']
        try:
            geometry=estimate(rgb,depth,snapshot,detection,name)
            output['geometry_candidates'].append(dict(detection=detection,geometry=geometry,
                identity_score_passed=detection['confidence']>=profile['identity_confidence']))
        except (KeyError,TypeError,ValueError) as exc:
            output['rejections'].append(dict(detection=detection,reason=str(exc)))
    output['classification_detection_count']=sum(d.get('name')==name for d in detections)
    output['unique_geometry_candidate']=len(output['geometry_candidates'])==1 and not output['rejections']
    if not output['classification_detection_count']:output['reason']='no_target_detection_not_proof_of_absence'
    output['source_sha256']={n:hashlib.sha256((capture/n).read_bytes()).hexdigest() for n in ('rgb.png','depth_m.npy','snapshot.json')}
    return output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--object',required=True,choices=('apple','rubiks_cube','canned_juice','ground_pepper'))
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--detections',type=Path,help='JSON list or an observation containing detections; no automatic model/version selection')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    detections=json.loads((args.detections or args.capture/'observation.json').read_text())
    if isinstance(detections,dict):detections=detections['detections']
    if not isinstance(detections,list):raise ValueError('explicit_detection_list_required')
    result=inspect(args.capture,detections,args.object)
    with args.output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()

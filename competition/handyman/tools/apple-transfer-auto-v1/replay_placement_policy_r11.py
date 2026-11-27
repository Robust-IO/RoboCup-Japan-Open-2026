"""Replan recorded table-depth frames with current policy. NEVER a live lease."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from apple_transfer_auto_plan import plan_placement,validate_config
from measure_destination_surface import measure


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording',type=Path,required=True)
    p.add_argument('--verified-held',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    config=validate_config(json.loads(a.config.read_text()))
    table=next(t for t in json.loads(Path(config['semantic_map']).read_text())['destinations']
        if t['id']==config['destination_id'])
    held=json.loads(a.verified_held.read_text())
    frames=sorted(a.recording.glob('*/snapshot.json'))
    if not frames:raise ValueError('recorded_frames_required')
    original=[a.verified_held,*[file for frame in frames
        for file in (frame,frame.parent/'depth_m.npy',frame.parent/'rgb.png')]]
    prior_result=a.recording.parent/'result.json'
    if prior_result.exists():original.append(prior_result)
    hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in original}
    a.output.mkdir(parents=True,exist_ok=False)
    rows=[]
    for frame in frames:
        snapshot=json.loads(frame.read_text());depth=np.load(frame.parent/'depth_m.npy',allow_pickle=False)
        plane=measure(depth,snapshot,table)
        plan=plan_placement(depth,snapshot,table,plane,held,config,'RECORDED_ONLY_r11_no_live_authority')
        with (a.output/(frame.parent.name+'-plan.json')).open('x') as f:json.dump(plan,f,indent=2,allow_nan=False)
        rows.append(dict(capture=str(frame.parent),original_depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
            planned_approach_distance_m=plan['planned_approach_distance_m'],release_xy_map_m=plan['release_xy_map_m'],
            patch_clear=plan['patch']['clear'],table_edge_safety=plan['table_edge_safety']))
    if any(hashlib.sha256(Path(path).read_bytes()).hexdigest()!=value for path,value in hashes.items()):
        raise ValueError('original_recording_changed')
    result=dict(status='recorded_replanning_passed',frames=len(rows),rows=rows,original_hashes=hashes,
        original_recordings_and_failed_result_unchanged=True,robot_commands_sent=False,
        live_placement_verified=False,new_policy_live_test_performed=False,
        actual_staging_correction_performed=False,
        warning='Saved-depth planning only. Historical stamps unchanged, predicted arm reach is NOT a measured live held target or motion/release lease.')
    with (a.output/'result.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps({key:value for key,value in result.items() if key!='original_hashes'}),flush=True)


if __name__=='__main__':main()

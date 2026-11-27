"""Fresh read-only grasp/lift continuation audit; no publishers/actions."""
import argparse
import json
from pathlib import Path
import sys
import time

p=argparse.ArgumentParser();p.add_argument('--code',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
sys.path.insert(0,str(args.code))
import cv2
import numpy as np
import rclpy
from canned_grasp_runtime import CanRuntime
from canned_grasp_geometry import measure_near,lift_follow,transform_point
from canned_carry_geometry import current_images_and_feedback,carry_initial
from held_can_appearance import build_appearance
from apple_transfer_automation import RobotLock,write_json

base=Path('/__HANDYMAN_PACKAGE__/runtime');original=base/'can-auto-live-20261023-01/grasp';root=args.output;root.mkdir(parents=True,exist_ok=False)
read=lambda p:json.loads(p.read_text());r=read(original/'result.json');before=original/'captures/preclose'
s0=read(before/'snapshot.json');m0=read(before/'can-measurement.json');holding=r['close']['closure_evidence']['observed_opening_rad']
appearance=build_appearance(cv2.cvtColor(cv2.imread(str(before/'rgb.png')),cv2.COLOR_BGR2RGB),
    np.load(before/'depth_m.npy',allow_pickle=False),s0,m0)
c0=transform_point(s0,m0['center_m']);p0=transform_point(s0,s0['palm_base_at_depth'])
runtime=None;value=dict(status='starting',robot_commands_sent=False,old_result_overwritten=False)
with RobotLock():
    try:
        rclpy.init();runtime=CanRuntime(root,r['epoch'],lambda e:None);rows=[];rejected=[];end=time.monotonic()+12
        while len(rows)<3 and time.monotonic()<end:
            sample=runtime.sample(processing_reserve_ns=0);snap=sample['snapshot']
            try:
                p1=transform_point(snap,snap['palm_base_at_depth']);expected=transform_point(snap,c0+p1-p0,inverse=True)
                m=measure_near(sample['depth'],snap,expected,m0['footprint_radius_m'],m0['height_m'],rgb=sample['rgb'],appearance=appearance)
                proof=lift_follow(m0,s0,m,snap,holding);state=runtime.state()
                carry_initial(state['joints'],r['grasp_plan']['contact_start_lift_m'],holding)
                timing=current_images_and_feedback(snap,state,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
                if rows and m['stamp_ns']-rows[-1]['stamp_ns']<200_000_000:continue
                path=runtime.save(sample,f'current-held-{len(rows):02d}',m)
                rows.append(dict(capture=str(path),measurement=m,proof=proof,timing=timing,state=state,stamp_ns=m['stamp_ns']))
            except ValueError as exc:
                path=runtime.save(sample,f'rejected-{len(rejected):02d}');rejected.append(dict(reason=str(exc),capture=str(path)))
        if len(rows)!=3:raise ValueError('current_tilted_can_not_verified:'+str(rejected[-2:]))
        value.update(status='current_can_held_after_tilted_grasp_verified',rows=rows,rejected=rejected,
            original_grasp=str(original),holding_opening_rad=holding,last_state=runtime.state(),
            source_epoch=r['epoch'],can_grasp_lift_independently_verified=True)
    except Exception as exc:value.update(status='current_grasp_not_verified',reason=str(exc))
    finally:
        if runtime:runtime.shutdown()
        if rclpy.ok():rclpy.shutdown()
        write_json(root/'result.json',value)
print(json.dumps({k:v for k,v in value.items() if k not in ('rows','rejected')}),flush=True)
if value['status']!='current_can_held_after_tilted_grasp_verified':raise SystemExit(1)

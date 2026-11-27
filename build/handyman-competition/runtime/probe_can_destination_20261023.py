"""Fresh destination measurement and held-can check. No robot commands."""
import json
from pathlib import Path
import sys

code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
import cv2
import numpy as np
import rclpy
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_plan import plan_placement,map_point
from canned_grasp_runtime import CanRuntime
from canned_carry_geometry import current_images_and_feedback,tucked_holding
from held_can_appearance import build_appearance,assess
from measure_destination_surface import measure
from pregrasp_geometry import expected_horizontal_center

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'can-destination-measured-20261023-01';root.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text())
ref=read(base/'canned-grasp-lift-live-progress-20261023.json');previous=read(base/'canned-carry-navigation-progress-20261023.json')
if previous['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():raise ValueError('same_boot_required')
cfg=read(code/'config/apple_transfer_auto_v1.json');table=next(x for x in read(Path(cfg['semantic_map']))['destinations'] if x['id']==cfg['destination_id'])
before=read(base/'canned-carry-live-20261023-01/before-retract.json');source=Path(before['rows'][-1]['capture'])
appearance=build_appearance(cv2.cvtColor(cv2.imread(str(source/'rgb.png')),cv2.COLOR_BGR2RGB),
    np.load(source/'depth_m.npy',allow_pickle=False),read(source/'snapshot.json'),read(source/'can-measurement.json'))
held=dict(center_minus_palm_m=ref['held_center_minus_palm_base_m'],radius_m=ref['can_footprint_radius_m'],
    holding_opening_rad=ref['held_opening_rad'],bottom_below_palm_m=ref['estimated_upright_bottom_below_palm_m'])
runtime=None;report=dict(status='starting',robot_commands_sent=False,placement_verified=False)
with RobotLock():
    try:
        rclpy.init();runtime=CanRuntime(root,root.name,lambda event:None);rows=[]
        for i in range(3):
            sample=runtime.sample(processing_reserve_ns=0);s=sample['snapshot']
            evidence=assess(sample['rgb'],sample['depth'],s,ref,appearance)
            plane=measure(sample['depth'],s,table);state=runtime.state()
            timing=current_images_and_feedback(s,state,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
            if not tucked_holding(state['joints'],ref['held_opening_rad']):raise ValueError('current_can_not_tucked_held')
            path=runtime.save(sample,f'destination-{i:02d}')
            rows.append(dict(capture=str(path),plane=plane,held_evidence=evidence,timing=timing,current_feedback=state))
        plan=plan_placement(sample['depth'],s,table,plane,held,cfg,'can-placement-20261023-01')
        predicted=map_point(s,expected_horizontal_center(plan['palm_height_m'],held['center_minus_palm_m']))
        front=min(plane['observed_xy_min'][1],plan['footprint_center'][1]-plan['footprint_half_size'][1])
        outside=bool(predicted[1]+max(held['radius_m'],.08)<front-.03)
        report.update(status='current_held_can_and_destination_measured',rows=rows,held_geometry=held,
            table=table,placement_plan=plan,predicted_horizontal_center_map_m=predicted.tolist(),
            table_front_map_y_m=front,horizontal_preparation_outside_table=outside,last_state=runtime.state())
        write_json(root/'placement-plan.json',plan)
    except Exception as exc:report.update(status='measurement_not_ready',reason=str(exc))
    finally:
        if runtime:runtime.shutdown()
        if rclpy.ok():rclpy.shutdown()
        write_json(root/'result.json',report)
print(json.dumps({k:v for k,v in report.items() if k not in ('rows','table','placement_plan')}),flush=True)
if report['status']!='current_held_can_and_destination_measured':raise SystemExit(1)

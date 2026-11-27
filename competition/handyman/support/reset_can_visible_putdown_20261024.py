"""Explicit measured put-down, one metre retreat and empty-arm reset."""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_plan import map_point
from can_transfer_context import load_context
from canned_grasp_geometry import measure_near,fresh
from can_destination_surface import measure as table_measure
from canned_grasp_runtime import CanRuntime,NAMES,TOPICS
from reviewed_carry_adjustment import endpoint
from motion_speed import finite_duration
import rclpy

base=Path('/__HANDYMAN_PACKAGE__/runtime')
source=base/'can-extra-seat35-live-20261024-03'
root=base/'can-seat35-visible-putdown-20261024-01'
context=load_context(source/'held-context.json',source/'reset-audit')
reference=json.loads((source/'grasp/locked-can-reference.json').read_text())
cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
table=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==cfg['source_table_id'])
runtime=None
report=dict(status='starting',new_closure_commands_sent=0,release_commands_sent=0,retreat_commands_sent=0,
    automatic_action_retry=False,requested_retreat_m=.4,robot_commands_sent=False)
with RobotLock():
    root.mkdir(exist_ok=False)
    with (root/'journal.jsonl').open('x') as journal:
        def record(event):
            row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
        def stage(name):
            report['stage']=name;write_json(root/'result.json',report);record(dict(event='stage',stage=name))
        def move(topic,names,target,duration,label):
            state=runtime.state();runtime.ownership();pub=runtime.publisher(topic)
            try:
                runtime.ownership(topic);record(dict(event='finite_command_once',label=label,names=names,target=target,duration_s=duration))
                pub.publish(runtime.trajectory(names,target,duration));report['robot_commands_sent']=True
                if label=='release_supported_can':report['release_commands_sent']+=1
                if label=='clear_after_putdown':report['retreat_commands_sent']+=1
                sent=time.monotonic();current=None
                while time.monotonic()-sent<duration+.7:
                    runtime.pump(.02);current=runtime.state();runtime.ownership(topic)
                if topic==TOPICS[2] and max(abs(a-b) for a,b in zip(current['joints'][:5],target))>.005:
                    raise ValueError('arm_endpoint_not_observed:'+label)
                if topic==TOPICS[3] and abs(current['joints'][5]-target[0])>.008:
                    raise ValueError('opening_endpoint_not_observed:'+label)
                return current
            finally:runtime.node.destroy_publisher(pub)
        def observed_support(label):
            # Current cap depth and current table plane, never the old upright bottom.
            for attempt in range(5):
                sample=runtime.sample();snap=sample['snapshot']
                expected=np.asarray(snap['palm_base_at_depth'])+context['held_center_minus_palm_base_m']
                measured=measure_near(sample['depth'],snap,expected,context['can_footprint_radius_m'],
                    context['can_height_m'],rgb=sample['rgb'],appearance=reference['source_appearance'])
                plane=table_measure(sample['depth'],snap,table)
                center=map_point(snap,np.asarray(measured['center_m']))
                bottom=map_point(snap,np.asarray(measured['estimated_lowest_point_base_m']))
                b=table['axis_aligned_bounds'];bc=np.array([b['center']['x'],b['center']['y']]);half=np.array([b['size']['x'],b['size']['y']])/2
                margin=half-abs(center[:2]-bc)-measured['horizontal_support_radius_m']
                if min(margin)<.015:raise ValueError('current_can_footprint_not_fully_over_source_table')
                gap=float(bottom[2]-np.r_[bottom[:2],1.]@plane['plane_map'])
                try:fresh(snap,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
                except ValueError:
                    if attempt<4:continue
                    raise
                path=runtime.save(sample,label,measured)
                proof=dict(capture=str(path),bottom_gap_m=gap,footprint_margin_m=margin.tolist(),plane=plane,
                    center_map_m=center.tolist(),actual_tilt_deg=measured['tilt_deg'])
                write_json(path/'support.json',proof)
                return proof
        try:
            rclpy.init();runtime=CanRuntime(root,root.name,record)
            sample=runtime.sample();initial=runtime.state();report['initial_state']=initial
            if abs(initial['joints'][5]-context['held_opening_rad'])>.015:
                raise ValueError('current_grip_differs_from_verified_hold')
            stage('measure_actual_can_and_table_support')
            support=observed_support('before-putdown');report['before_support']=support
            target=initial['joints'][:5].copy();target[0]-=support['bottom_gap_m']-.002
            if not .07<=target[0]<=.443:raise ValueError('measured_lowering_outside_arm_range')
            stage('lower_to_measured_table')
            move(TOPICS[2],NAMES[:5],target,1.5,'lower_to_measured_table')
            support=observed_support('supported-before-release');report['release_support']=support
            if not -.005<=support['bottom_gap_m']<=.012:
                raise ValueError('can_not_yet_at_measured_table_support_no_release')
            stage('release_supported_can')
            move(TOPICS[3],[NAMES[5]],[.85],2.,'release_supported_can')
            runtime.save(runtime.sample(),'after-release')
            stage('retreat_after_putdown')
            current=runtime.state();start=current['poses']['odom'];delta=[-.4,0.,0.]
            target_base=endpoint(start,delta,max_translation_m=None)
            duration=finite_duration(runtime.profile,delta,max_translation_m=None)
            end=move(TOPICS[1],['odom_x','odom_y','odom_t'],target_base,duration,'clear_after_putdown')
            report['actual_retreat_m']=math.dist(start[:2],end['poses']['odom'][:2])
            report['retreat_endpoint_error_m']=math.dist(target_base[:2],end['poses']['odom'][:2])
            if report['retreat_endpoint_error_m']>.025:raise ValueError('retreat_endpoint_not_observed')
            stage('tuck_empty_arm')
            arm=runtime.state()['joints'][:5].copy()
            for index,value,label in ((1,0.,'fold_empty_elbow'),(3,-math.pi/2,'restore_empty_wrist'),(0,0.,'lower_empty_arm')):
                arm[index]=value;move(TOPICS[2],NAMES[:5],arm.copy(),1.5,label)
            move(TOPICS[3],[NAMES[5]],[.65],1.2,'normalize_empty_opening')
            report['final_capture']=str(runtime.save(runtime.sample(),'empty-tucked-clear-of-table'))
            report.update(status='measured_putdown_and_retreat_completed',hand_empty=True,robot_tucked=True)
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
        finally:
            if runtime:
                try:report['last_fresh_robot_state']=runtime.state()
                except Exception as exc:report['state_read_error']=str(exc)
                runtime.shutdown()
            if rclpy.ok():rclpy.shutdown()
            write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if report['status']!='measured_putdown_and_retreat_completed':raise SystemExit(1)

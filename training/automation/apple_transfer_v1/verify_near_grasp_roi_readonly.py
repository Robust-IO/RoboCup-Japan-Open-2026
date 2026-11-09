"""Verify fresh ROI identity at trial05's prepared pose. NO robot actions.

This is NOT the empty-tucked full-transfer preflight, recovery or a grasp lease.
It starts a NEW task/epoch with no old identity and enables ROI only in memory.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid

from apple_transfer_auto_backend import LiveBackend,read
from apple_transfer_auto_plan import fresh_snapshot,map_pose,plan_grasp
from apple_transfer_automation import check_inputs,write_json
from apple_roi_confirmation import identity_evidence
from apple_approach_lease import validate_closure

ARM=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint')


def check_prepared_snapshot(snapshot,expected):
    values=snapshot['joints'];arm=expected['arm'];pose=map_pose(snapshot)
    if (len(arm)!=5 or not .24<=arm[0]<=.30
            or any(abs(x-y)>.006 for x,y in zip(arm[1:],[-1.7,0,.1292036732051034,0]))
            or expected.get('holding_object') is not False or abs(expected['hand_motor']-.65)>.001):
        raise ValueError('reviewed_prepared_empty_reference_required')
    checked=[values[k] for k in (*ARM,'hand_motor_joint','head_pan_joint','head_tilt_joint')]
    if (not all(math.isfinite(v) for v in checked)
            or any(abs(values[k]-v)>.006 for k,v in zip(ARM,arm))
            or abs(values['hand_motor_joint']-.65)>.005
            or abs(values['head_pan_joint'])>.006 or abs(values['head_tilt_joint']+.5)>.006):
        raise ValueError('prepared_open_pose_changed')
    before=expected['base_map_pose']
    if (math.dist(pose[:2],before[:2])>.01
            or abs(math.atan2(math.sin(pose[2]-before[2]),math.cos(pose[2]-before[2])))>.02):
        raise ValueError('base_moved_from_reviewed_prepared_pose')
    return pose


def main():
    import rclpy
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--confirm-prepared-pose',action='store_true')
    parser.add_argument('--samples',type=int,default=15)
    parser.add_argument('--config',type=Path,default=Path(__file__).parent/'config/apple_transfer_auto_v1.json')
    parser.add_argument('--checkpoint',type=Path,default=Path('/mnt/d/handyman-training-runs/apple-auto05-validation-progress-20261017.json'))
    args=parser.parse_args()
    if (not args.confirm_prepared_pose or not 5<=args.samples<=30
            or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or not args.output.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))):
        parser.error('Explicit prepared simulation pose, domain71 localhost and NEW D-drive output required')
    args.output.mkdir(parents=True,exist_ok=False)
    reference=read(args.checkpoint)
    if reference['trial']!='automatic-apple-20261017-05' or reference['actual_approach']['closure_command_sent'] is not False:
        raise ValueError('reviewed_trial05_preclosure_reference_required')
    expected=reference['post_stop_state'];config=read(args.config)
    config['policy']['near_grasp_roi_confirmation']=True
    inputs=check_inputs(config);write_json(args.output/'input-files-audit.json',inputs)
    write_json(args.output/'config.snapshot.json',config)
    task='near-roi-'+uuid.uuid4().hex;rows=[];backend=None
    result=dict(status='stopped',robot_commands_sent=False,grasp_commands_sent=False,
                whole_transfer_verified=False,production_config_changed=False,
                scope='NEW fresh ROI identity and production source/closure geometry while prepared and stationary ONLY')
    log=(args.output/'journal.jsonl').open('x')
    def record(value):log.write(json.dumps(dict(time_ns=time.time_ns(),**value))+'\n');log.flush()
    def forbidden(*_args,**_kwargs):raise RuntimeError('robot_actions_forbidden_in_roi_readonly_validation')
    rclpy.init();node=rclpy.create_node('near_grasp_roi_graph_readonly')
    topics=['/hsrb/'+tail for tail in ('command_velocity','omni_base_controller/command',
            'arm_trajectory_controller/command','gripper_controller/command','head_trajectory_controller/command')]
    def graph_check():
        counts={topic:node.count_publishers(topic) for topic in topics}
        if any(counts.values()):raise ValueError('other_robot_command_publisher_present')
        if any(node.count_subscribers(topic)!=1 for topic in topics[1:]):raise ValueError('robot_command_receiver_missing_or_ambiguous')
        return counts
    try:
        end=time.monotonic()+4
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        result['publishers_before']=graph_check()
        backend=LiveBackend(config,inputs);backend.bind(args.output,task,record)
        backend.stage='near_grasp_roi_readonly'
        backend.command=backend.arm=backend.adjust=backend.navigate=forbidden
        backend.start_observer('source-observe',True)
        end=time.monotonic()+120;seen=set()
        while len(rows)<args.samples:
            if time.monotonic()>=end:raise RuntimeError('fresh_roi_sample_budget_expired')
            capture,snapshot,observation,evidence=backend.latest(100 if not rows else 8)
            if str(capture) in seen:time.sleep(.02);continue
            pose=check_prepared_snapshot(snapshot,expected);graph_check()
            roi=identity_evidence(observation,snapshot)
            if roi is None or evidence is None:raise ValueError('fresh_roi_identity_required')
            plan=plan_grasp(observation,snapshot,backend.tables[config['source_table_id']],evidence,config)
            odom_tf=snapshot['tracking_transforms']['odom_from_base_at_depth']
            from hand_camera_ray import rotation
            r=rotation(odom_tf['quaternion']);odom=[*odom_tf['translation'][:2],math.atan2(r[1,0],r[0,0])]
            closure_ok=True;closure_reason=None
            try:validate_closure(observation,snapshot,now_ns=time.time_ns(),epoch=backend.epoch,
                    task_id=task,current_odom=odom,palm_height=plan['palm_height_m'],expected_room='bedroom')
            except ValueError as exc:closure_ok=False;closure_reason=str(exc)
            now=time.time_ns();fresh_snapshot(snapshot,now,backend.epoch)
            row=dict(capture=str(capture),depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
                joint_age_s=(now-snapshot['joint_stamp_ns'])/1e9,
                image_age_s=(now-min(snapshot['streams'][key]['stamp_ns'] for key in ('rgb','depth')))/1e9,
                rgb_depth_delta_s=abs(snapshot['streams']['rgb']['stamp_ns']-snapshot['streams']['depth']['stamp_ns'])/1e9,
                detector_confidence=roi['full_frame_confidence'],roi_confidence=roi['confidence'],
                measured_target_base_m=evidence['target_base_m'],base_map_pose=pose,
                grasp_plan_eligible=True,closure_region_passed=closure_ok,closure_rejection=closure_reason,
                palm_height_m=plan['palm_height_m'],pipeline_timing=observation['timing'])
            rows.append(row);seen.add(str(capture));print(json.dumps(dict(sample=len(rows),**row)),flush=True)
        backend.close();backend=None
        for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
        result.update(status='near_grasp_roi_readonly_verified',samples=len(rows),
            publishers_after=graph_check(),min_roi_confidence=min(row['roi_confidence'] for row in rows),
            max_joint_age_s=max(row['joint_age_s'] for row in rows),max_image_age_s=max(row['image_age_s'] for row in rows),
            all_grasp_plans_eligible=True,all_closure_regions_passed=all(row['closure_region_passed'] for row in rows),
            original_geometry_and_stamps_preserved=True,freshness_or_identity_thresholds_changed=False)
    except (Exception,KeyboardInterrupt) as exc:result['reason']=str(exc) or 'interrupted'
    finally:
        if backend is not None:backend.close()
        node.destroy_node();rclpy.shutdown();log.close();result['rows']=rows
        write_json(args.output/'result.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
    if result['status']!='near_grasp_roi_readonly_verified':raise SystemExit(1)


if __name__=='__main__':main()

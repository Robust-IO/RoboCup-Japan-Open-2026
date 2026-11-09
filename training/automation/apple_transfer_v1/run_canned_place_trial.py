"""Explicit can placement from already tucked/held living-room state only."""
import argparse
import json
import os
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path)
    p.add_argument('--context',type=Path)
    p.add_argument('--owner-lock-fd',type=int);p.add_argument('--owner-run-root',type=Path)
    p.add_argument('--release-only-from',type=Path)
    args=p.parse_args()
    if not args.run:
        print(json.dumps(dict(status='dry_can_placement_plan',robot_commands_sent=False,grasp_supported=False,
            navigation_supported=False,stages=['fresh_destination','prepare','full_approach','release_once','withdraw','verify_placement'])));return
    if (args.output is None or not str(args.output).startswith('/mnt/d/handyman-training-runs/')
            or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):
        p.error('explicit domain71/localhost and NEW D-drive output required')
    import cv2
    import numpy as np
    import rclpy
    from apple_transfer_automation import RobotLock,check_inputs,write_json
    from apple_transfer_auto_backend import LiveBackend
    from apple_transfer_auto_plan import map_pose,map_point
    from canned_carry_geometry import current_held,current_images_and_feedback,tucked_holding
    from canned_grasp_geometry import measure_near
    from canned_grasp_runtime import CanRuntime,TOPICS
    from canned_place_geometry import make_plan,approach_step,region,body_appearance,placed_expected,verify_placed
    from held_can_appearance import build_appearance,assess
    from inverse_table_place_plan import OFFSET,FLEX,WRIST
    from can_destination_surface import measure
    from motion_speed import arm_duration
    from pregrasp_geometry import expected_horizontal_center
    from task_outcome_policy import pose_matches,hand_is_open,robot_ready
    from can_transfer_context import stage_lock,load_context
    from can_release_timing import publish_once,reviewed_plan
    base=Path('/mnt/d/handyman-training-runs');code=Path(__file__).parent;root=args.output;root.mkdir(exist_ok=False,parents=True)
    read=lambda path:json.loads(path.read_text())
    if args.context:ref=load_context(args.context,root,require_carried=True)
    else:
        ref=read(base/'canned-grasp-lift-live-progress-20261023.json');previous=read(base/'canned-carry-navigation-progress-20261023.json')
        if not previous.get('holding_after_navigation_verified') or previous['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
            raise ValueError('same_boot_verified_can_carry_required')
    cfg=read(code/'config/apple_transfer_auto_v1.json');table=next(x for x in read(Path(cfg['semantic_map']))['destinations'] if x['id']==cfg['destination_id'])
    source=Path(ref['source_capture']) if args.context else Path(read(base/'canned-carry-live-20261023-01/before-retract.json')['rows'][-1]['capture'])
    appearance=build_appearance(cv2.cvtColor(cv2.imread(str(source/'rgb.png')),cv2.COLOR_BGR2RGB),
        np.load(source/'depth_m.npy',allow_pickle=False),read(source/'snapshot.json'),read(source/'can-measurement.json'))
    report=dict(status='starting',placement_success_verified=False,hand_released=False,release_commands_sent=0,
        grasp_or_navigation_replayed=False,task_finished_sent=False,cleanup_completed=False,automatic_action_retry=False)
    report['context_run_id']=ref.get('run_id')
    runtime=None;backend=None;serial=0;plan=None
    def save():write_json(root/'result.json',report)
    with stage_lock(args.owner_lock_fd,args.owner_run_root),(root/'journal.jsonl').open('x') as stream:
        def record(event):
            value=dict(time_ns=time.time_ns(),**event);stream.write(json.dumps(value,allow_nan=False)+'\n');stream.flush()
            print(json.dumps(value),flush=True)
        def stage(name):report['stage']=name;save();record(dict(event='stage',stage=name))
        try:
            write_json(root/'input-audit.json',check_inputs(cfg));rclpy.init();runtime=CanRuntime(root,root.name,record)
            backend=LiveBackend(cfg);backend.bind(root,root.name,record);holding=ref['held_opening_rad']
            def observe(mode,label):
                nonlocal serial
                reason='no_current_can_surface'
                for attempt in range(5):
                    sample=runtime.sample(processing_reserve_ns=100_000_000 if mode!='tucked' else 0);s=sample['snapshot'];serial+=1
                    try:
                        plane=measure(sample['depth'],s,table)
                        if mode=='tucked':
                            evidence=assess(sample['rgb'],sample['depth'],s,ref,appearance);measurement=None
                        else:
                            if mode=='held':
                                evidence=current_held(sample['depth'],s,ref,rgb=sample['rgb'],appearance=appearance);measurement=evidence['measurement']
                            else:
                                measurement=measure_near(sample['depth'],s,placed_expected(s,plan),
                                    plan.get('surface_radius_m',plan['radius_m']),plan['height_m'],rgb=sample['rgb'],appearance=appearance)
                                evidence=dict(actual_placed_surface_measured=True)
                            evidence['appearance']=body_appearance(sample['rgb'],sample['depth'],s,measurement,appearance)
                        state=runtime.state();timing=current_images_and_feedback(s,state,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
                        if mode=='tucked' and not tucked_holding(state['joints'],holding):raise ValueError('current_can_not_tucked_held')
                        path=runtime.save(sample,f'{serial:03d}-{label}',measurement)
                        row=dict(measurement=measurement,snapshot=s,plane=plane,evidence=evidence,timing=timing,current_feedback=state,capture=str(path))
                        write_json(path/'observation.json',row);return sample,row
                    except ValueError as exc:
                        reason=str(exc);path=runtime.save(sample,f'{serial:03d}-{label}-rejected')
                        record(dict(event='new_frame_rejected_no_action_replayed',reason=reason,capture=str(path)))
                raise ValueError('can_placement_observation_failed:'+reason)
            def arm(name,which,row):
                return runtime.child([code/'pregrasp_raised_trial.py','--run','--segmented-high','--simulation-contact-trial',
                    '--reviewed-map-pose',*map_pose(row['snapshot']),'--motion-profile',cfg['motion_profile'],
                    '--inverse-place-stage',which,'--place-palm-height',plan['palm_height_m'],
                    '--holding-opening',holding,'--efficient-transfer'],name)
            def pose_observation(label):
                nonlocal serial
                sample=runtime.sample(processing_reserve_ns=0);s=sample['snapshot'];state=runtime.state();serial+=1
                timing=current_images_and_feedback(s,state,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
                path=runtime.save(sample,f'{serial:03d}-{label}')
                return dict(snapshot=s,current_feedback=state,timing=timing,capture=str(path),
                    object_reidentification_required=False,purpose='current_pose_for_empty_hand_cleanup')
            if args.release_only_from:
                if not args.context:raise ValueError('current_run_context_required_for_reviewed_release')
                plan=reviewed_plan(args.release_only_from,ref['run_id'],ref['run_root'])
                report.update(run_kind='reviewed_release_only_continuation',source_placement=str(args.release_only_from),
                    prepare_or_approach_replayed=False)
                write_json(root/'placement-plan.json',plan)
                stage('verify_current_pre_release_pose');sample,row=observe('held','reviewed-current-held')
                expected=[plan['palm_height_m']-OFFSET,FLEX,0.,WRIST,0.,holding]
                if not pose_matches(row['current_feedback']['joints'],expected):raise ValueError('current_reviewed_release_pose_changed')
                if not region(row['measurement'],row['snapshot'],plan,row['plane'])['ready']:
                    raise ValueError('current_reviewed_release_support_not_ready')
            else:
                stage('fresh_destination_and_held_can');sample,row=observe('tucked','destination')
                if backend.room_at(row['current_feedback']['poses']['map'])!='living_room':raise ValueError('current_robot_not_in_living_room')
                plan=make_plan(sample['depth'],row['snapshot'],table,row['plane'],ref,cfg,root.name)
                write_json(root/'placement-plan.json',plan)
                predicted=map_point(row['snapshot'],expected_horizontal_center(plan['palm_height_m'],ref['held_center_minus_palm_base_m']))
                front=min(row['plane']['observed_xy_min'][1],plan['footprint_center'][1]-plan['footprint_half_size'][1])
                if predicted[1]+max(plan['radius_m'],.08)>=front-.03:raise ValueError('horizontal_preparation_not_outside_table')
                stage('prepare_horizontal_above_table');report['prepare']=arm('place-prepare','prepare',row);save()
                stage('full_measured_table_approach');backend.stage='approach_placement';steps=[]
                for index in range(6):
                    sample,row=observe('held',f'held-approach-{index:02d}')
                    step=approach_step(row['measurement'],row['snapshot'],plan,row['plane'])
                    if step['aligned']:break
                    if index==5:raise ValueError('five_placement_corrections_exhausted')
                    current=row['current_feedback'];steps.append(dict(delta_m=step['delta_m'],before_capture=row['capture']))
                    report['approaches']=steps;save()
                    backend.adjust(f'placement-approach-{index:02d}',current['poses']['map'],step['delta_m'],
                        current['joints'][:5],holding,single_placement_approach=True)
            stage('release_once_from_current_rgbd');topic=TOPICS[3];pub=runtime.publisher(topic)
            try:
                def acquire_release():
                    sample,row=observe('held','pre-release');check=region(row['measurement'],row['snapshot'],plan,row['plane'])
                    if not check['ready']:raise ValueError('actual_can_not_over_safe_table_support')
                    state=row['current_feedback'];before=state['joints'];goal=before.copy();goal[5]=.65
                    duration=arm_duration(runtime.profile,'hand',before,goal,holding=True,segmented=True)
                    message=runtime.trajectory(['hand_motor_joint'],[.65],duration)
                    record(dict(event='can_release_intent',capture=row['capture'],region=check,duration_s=duration))
                    runtime.ownership(topic)
                    current_images_and_feedback(row['snapshot'],state,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
                    return row,check,state,before,duration,message
                def publish_release(value):
                    pub.publish(value[-1]);return value
                row,check,state,before,duration,_=publish_once(acquire_release,publish_release,
                    lambda index,reason:record(dict(event='expired_release_frame_reacquire_without_action',index=index,reason=reason)))
                report['release_commands_sent']=1;report['release_geometry']=check
                plan['actual_release_xy_map_m']=check['center_map_m'][:2]
                write_json(root/'actual-release-geometry.json',check);save()
                record(dict(event='can_release_sent',capture=row['capture']))
                sent=time.monotonic();stable=None;last=state['stamp_ns']
                while time.monotonic()-sent<duration+5:
                    runtime.pump(.02);state=runtime.state();runtime.ownership(topic)
                    if not pose_matches(state['joints'][:5]+[holding],before):raise ValueError('arm_changed_during_can_release')
                    if state['stamp_ns']<=last:continue
                    last=state['stamp_ns']
                    if hand_is_open(state['joints'][5]):
                        if stable is None:stable=time.monotonic()
                        if time.monotonic()-sent>=duration and time.monotonic()-stable>=.6:break
                    else:stable=None
                else:raise ValueError('can_hand_open_feedback_not_settled')
                report['hand_released']=True;save()
            finally:runtime.node.destroy_publisher(pub)
            # Empty-hand cleanup needs current pose, not repeated recognition
            # through the fingers. Actual placed-object proof follows withdrawal.
            stage('withdraw_open_hand');backend.stage='withdraw'
            try:
                row=pose_observation('before-raise');report['raise']=arm('place-raise','raise',row)
                row=pose_observation('after-raise')
                current=row['current_feedback']
                backend.adjust('placement-retreat',current['poses']['map'],[-cfg['policy']['retreat_m'],0.,0.],
                    current['joints'][:5],current['joints'][5])
                row=pose_observation('after-retreat');report['retract']=arm('place-retract','retract',row)
                report['cleanup_completed']=True
            except Exception as exc:
                report['cleanup_error']=str(exc);report['robot_ready_for_next_task']=False
                record(dict(event='cleanup_stopped_verify_actual_placement_readonly',reason=str(exc)))
            stage('verify_actual_can_placement');rows=[]
            while len(rows)<3:
                sample,row=observe('placed',f'final-{len(rows):02d}')
                if rows and row['measurement']['stamp_ns']-rows[-1]['measurement']['stamp_ns']<200_000_000:continue
                rows.append(row)
            state=runtime.state();verification=verify_placed(rows,plan,state['joints'],cleanup_completed=report['cleanup_completed'])
            write_json(root/'placement-verification.json',verification)
            report.update(status='reviewed_can_placement_verified',placement_success_verified=True,
                robot_ready_for_next_task=bool(report['cleanup_completed'] and robot_ready(dict(zip(
                    ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint'),state['joints'])))),
                final_placement_rows=[dict(capture=r['capture'],measurement=r['measurement']) for r in rows],
                physical_can_transfer_completed=True,uninterrupted_full_automation_verified=False)
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped_requires_current_state_review',reason=str(exc) or 'interrupted',automatic_resume=False)
        finally:
            if backend:backend.close()
            if runtime:
                try:report['last_fresh_robot_state']=runtime.state()
                except Exception as exc:report['state_read_error']=str(exc)
                runtime.shutdown()
            if rclpy.ok():rclpy.shutdown()
            save();print(json.dumps(report),flush=True)
    if report['status']!='reviewed_can_placement_verified':raise SystemExit(1)


if __name__=='__main__':main()

"""Explicit fixed-source can grasp/lift trial, not a complete transfer.

Default: print a dry plan. --observe-only: new model/RGBD observations without
publishers. --run --simulation-ignore-collision: prepare, one coarse approach,
at most five new measured corrections, close ONCE, then verify a 10cm lift.
Never retries a failed action, opens a held hand, navigates, or sends protocol.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

MAX_CORRECTIONS=5
SOURCE_IMGSZ=640


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--observe-only',action='store_true')
    p.add_argument('--simulation-ignore-collision',action='store_true');p.add_argument('--output',type=Path)
    p.add_argument('--owner-lock-fd',type=int);p.add_argument('--owner-run-root',type=Path)
    p.add_argument('--require-upright',action='store_true',help='Upright-can experiment: observe real cap axis before closure')
    p.add_argument('--extra-seat-m',type=float,default=0.,help='Can-only experiment: advance this much beyond the calibrated grasp center before closing (metres)')
    p.add_argument('--record-close-pose',action='store_true',help='Save three diagnostic RGBD can poses after closure and before lift')
    args=p.parse_args()
    if not math.isfinite(args.extra_seat_m) or not 0.<=args.extra_seat_m<=.05:
        p.error('extra-seat-m must be a finite distance from 0 to .05 metres')
    if args.run and args.observe_only:p.error('choose observation or explicit grasp')
    if not args.run and not args.observe_only:
        print(json.dumps(dict(status='dry_plan',class_name='canned_juice',source='LayoutA bedroom white_side_table',
            robot_commands_sent=False,one_coarse_approach=True,maximum_new_measured_corrections=MAX_CORRECTIONS,
            source_detector_imgsz=SOURCE_IMGSZ,
            close_commands_maximum=1,placement_or_transfer_verified=False,
            extra_seat_m=args.extra_seat_m,record_close_pose=args.record_close_pose,
            stages=['new_detection_rgbd','prepare_horizontal','new_depth_approach','one_close','lift','verify_depth_follow'])))
        return
    if (args.output is None or os.environ.get('ROS_DOMAIN_ID')!='71'
            or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or args.run and not args.simulation_ignore_collision):
        p.error('requires new output, reviewed domain71/localhost, and explicit simulation override for movement')
    import numpy as np
    import yaml
    from apple_transfer_automation import RobotLock,check_inputs,write_json
    from can_transfer_context import stage_lock
    from apple_transfer_auto_plan import footprint,map_point
    from canned_grasp_geometry import (aligned,anchor,fresh,grasp_plan,lift_follow,
        measure_near,predicted_before_contact,transform_point)
    from can_identity_selection import confirmed_cans
    from object_surface_geometry import estimate
    from task_outcome_policy import pose_matches
    import scipy.sparse.csgraph  # Warm geometry imports BEFORE selecting source frames.
    code=Path(__file__).parent;root=args.output;root.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(code/'scripts'))
    from scripts.search_room_gate import inside
    from apple_worker_client import Worker
    from canned_grasp_runtime import CanRuntime
    import rclpy
    cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
    env=yaml.safe_load(Path(cfg['environment']).read_text())
    semantic=json.loads(Path(cfg['semantic_map']).read_text())
    source_table=next(d for d in semantic['destinations'] if d['id']==cfg['source_table_id'])
    table_center,table_half=footprint(source_table)
    task=root.name
    run_identity=json.loads((args.owner_run_root/'run-meta.json').read_text())['run_id'] if args.owner_run_root else task
    epoch=run_identity+'-rgbd';runtime=None;worker=None
    report=dict(status='starting',class_name='canned_juice',task_id=task,epoch=epoch,
        mode='live_grasp_trial' if args.run else 'observation_only',grasp_success_verified=False,
        placement_verified=False,physical_transfer_verified=False,task_finished_sent=False,
        automatic_action_retry=False,maximum_new_measured_corrections=MAX_CORRECTIONS,
        navigation_or_model_changed=False,approaches=[])
    report['source_detector_imgsz']=SOURCE_IMGSZ
    def save():write_json(root/'result.json',report)
    with stage_lock(args.owner_lock_fd,args.owner_run_root),(root/'journal.jsonl').open('x') as journal:
        def record(event):
            value=dict(time_ns=time.time_ns(),**event)
            journal.write(json.dumps(value,allow_nan=False)+'\n');journal.flush()
            print(json.dumps(value,allow_nan=False),flush=True)
        def stage(name):
            report['stage']=name;save();record(dict(event='stage',stage=name))
        try:
            write_json(root/'input-audit.json',check_inputs(cfg))
            rclpy.init();runtime=CanRuntime(root,epoch,record,extra_seat_m=args.extra_seat_m)
            stage('warm_current_detector')
            worker=Worker(cfg['model_python'],cfg['registry'],root/'model-worker.log',imgsz=SOURCE_IMGSZ)
            registry=json.loads(Path(cfg['registry']).read_text())
            if worker.ready.get('model_sha256')!=registry['sha256']:raise ValueError('can_current_model_mismatch')
            stage('new_detection_rgbd')
            measurements=[];initial=None
            for index in range(9):
                sample=runtime.sample();capture=runtime.save(sample,f'identity-{index:02d}')
                save_complete_ns=time.time_ns()
                classification=worker.infer(capture,pump=lambda:runtime.pump(0.))
                inference_complete_ns=time.time_ns()
                write_json(capture/'raw-detector-result.json',classification)
                # Reuse the worker's 30-class detections ONLY. Its apple proposal
                # verdict, red mask, apple candidate and identity are never used.
                targets=confirmed_cans(classification.get('detections',[]))
                if len(targets)!=1:
                    record(dict(event='unconfirmed_source_frame_reacquire_without_action',capture=str(capture),
                        current_high_confidence_can_count=len(targets),imgsz=SOURCE_IMGSZ))
                    continue
                geometry=estimate(sample['rgb'],sample['depth'],sample['snapshot'],targets[0],'canned_juice')
                checked_ns=runtime.node.get_clock().now().nanoseconds
                try:fresh(sample['snapshot'],checked_ns,epoch)
                except ValueError as exc:
                    record(dict(event='discard_expired_observation_no_action',reason=str(exc),
                        capture=str(capture),checked_ns=checked_ns,save_complete_ns=save_complete_ns,
                        inference_complete_ns=inference_complete_ns,
                        source_stamp_ns=min(sample['snapshot']['streams'][k]['stamp_ns'] for k in ('rgb','depth'))))
                    continue
                mapped=map_point(sample['snapshot'],np.asarray(geometry['center_m']))
                if not inside(*mapped[:2],env['rooms']['bedroom']['region']):raise ValueError('can_outside_requested_bedroom')
                if np.any(np.abs(mapped[:2]-table_center)>table_half):raise ValueError('can_not_on_requested_source_table')
                item=dict(capture=str(capture),detection=targets[0],geometry=geometry,
                    center_odom_m=transform_point(sample['snapshot'],geometry['center_m']).tolist())
                write_json(capture/'can-identity.json',item);measurements.append(item);initial=(sample,geometry)
                if len(measurements)==3:break
            if len(measurements)!=3:raise ValueError('three_new_fresh_can_observations_required')
            centers=np.array([row['center_odom_m'] for row in measurements])
            if np.max(np.linalg.norm(centers-centers.mean(0),axis=1))>.015:raise ValueError('can_initial_centers_not_repeatable')
            source_sample,geometry=initial;reference=anchor(geometry,source_sample['snapshot'],task_id=task)
            from held_can_appearance import build_appearance
            reference['source_appearance']=build_appearance(source_sample['rgb'],source_sample['depth'],
                source_sample['snapshot'],geometry)
            reference['source_appearance_capture']=measurements[-1]['capture']
            reference['extra_seat_m']=args.extra_seat_m
            report['extra_seat_m']=args.extra_seat_m
            if args.require_upright:reference.update(require_upright_trial=True,maximum_preclose_tilt_deg=8.)
            report.update(initial_observations=measurements,grasp_plan=grasp_plan(geometry),reference=reference)
            write_json(root/'locked-can-reference.json',reference)
            if args.observe_only:
                report.update(status='new_can_observation_verified',robot_commands_sent=False)
                return
            state=runtime.state();empty=state['joints']
            if not pose_matches(empty[:5]+[0.],[0.,0.,0.,-math.pi/2,0.,0.]) or min(abs(empty[5]),abs(empty[5]-.65))>.03:
                raise ValueError('can_trial_requires_empty_tucked_start')
            lift=reference['contact_start_lift_m']
            common=[code/'pregrasp_raised_trial.py','--run','--segmented-high','--simulation-contact-trial',
                '--bedroom-side-table-grasp','--motion-profile','balanced_v1','--efficient-transfer']
            stage('prepare_horizontal')
            prepare=[*common,'--reviewed-map-pose',*state['poses']['map'],'--white-table-prepare-lift',lift]
            if empty[5]>.6:prepare.append('--prepare-hand-already-open')
            report['preparation']=runtime.child(prepare,'prepare')
            expected_arm=[lift,-1.7,0.,.1292036732051034,0.]
            def observe():
                return runtime.observe_near(reference,task)
            stage('measured_approach')
            for index in range(MAX_CORRECTIONS+1):
                sample,measured=observe()
                if aligned(measured['center_m'],sample['snapshot'],extra_seat_m=args.extra_seat_m):break
                result=runtime.approach(observe,expected_arm,index)
                report['approaches'].append(result);save()
            sample,measured=observe()
            if not aligned(measured['center_m'],sample['snapshot'],extra_seat_m=args.extra_seat_m):raise RuntimeError('can_alignment_not_reached_after_five_corrections')
            stage('one_closure')
            close,before_sample,before_geometry=runtime.close_gripper(observe,expected_arm)
            report['close']=close;save()
            opening=close['closure_evidence']['observed_opening_rad']
            if not .02<opening<=.62:raise RuntimeError('can_no_verified_contact_no_lift')
            if args.record_close_pose:
                stage('observe_closed_pose_before_lift')
                report['postclose_pose_observations']=runtime.record_closed_pose(reference,task)
                save()
            stage('verification_lift')
            state=runtime.state()
            report['lift']=runtime.child([*common,'--reviewed-map-pose',*state['poses']['map'],
                '--measured-contact-start-lift',lift,'--contact-finish','lift','--holding-opening',opening],'lift')
            completed=time.time_ns();stage('verify_current_can_follows_palm')
            c0=transform_point(before_sample['snapshot'],before_geometry['center_m'])
            p0=transform_point(before_sample['snapshot'],before_sample['snapshot']['palm_base_at_depth'])
            from held_can_appearance import build_appearance
            held_appearance=build_appearance(before_sample['rgb'],before_sample['depth'],before_sample['snapshot'],before_geometry)
            accepted=[];rejected=[];deadline=time.monotonic()+8
            while len(accepted)<3 and time.monotonic()<deadline:
                sample=runtime.sample();snap=sample['snapshot']
                if min(snap['streams'][k]['stamp_ns'] for k in ('rgb','depth'))<=completed:continue
                try:
                    p1=transform_point(snap,snap['palm_base_at_depth'])
                    expected=transform_point(snap,c0+p1-p0,inverse=True)
                    measured=measure_near(sample['depth'],snap,expected,reference['radius_m'],reference['height_m'],
                        rgb=sample['rgb'],appearance=held_appearance)
                    proof=lift_follow(before_geometry,before_sample['snapshot'],measured,snap,opening)
                    fresh(snap,runtime.node.get_clock().now().nanoseconds,epoch)
                    if accepted and proof['stamp_ns']-accepted[-1]['stamp_ns']<200_000_000:continue
                    path=runtime.save(sample,f'after-lift-{len(accepted):02d}',measured)
                    accepted.append(dict(proof,capture=str(path)))
                except ValueError as exc:
                    runtime.save(sample,f'after-lift-rejected-{len(rejected):02d}')
                    rejected.append(str(exc))
            if len(accepted)!=3:raise RuntimeError('can_lift_follow_not_verified:'+str(rejected[-3:]))
            relatives=np.array([row['center_minus_palm_odom_m'] for row in accepted])
            if np.max(np.linalg.norm(relatives-relatives.mean(0),axis=1))>.02:raise RuntimeError('can_not_stable_relative_to_palm')
            report.update(status='can_grasp_and_lift_verified',grasp_success_verified=True,
                held_opening_rad=opening,lift_verification=accepted,rejected_measurements=rejected,
                robot_commands_sent=True,can_remains_held=True,arm_retracted=False)
            if args.require_upright:
                report['upright_trial_selected']=True
                report['verified_preclose_tilt_deg']=before_geometry['tilt_deg']
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted',
                automatic_resume=False,hand_released_by_cleanup=False)
        finally:
            if runtime:
                try:report['last_fresh_robot_state']=runtime.state()
                except Exception as exc:report['final_state_read_error']=str(exc)
            if worker:worker.close()
            if runtime:runtime.shutdown()
            if rclpy.ok():rclpy.shutdown()
            save();print(json.dumps(report,allow_nan=False),flush=True)
    if report['status'] not in ('can_grasp_and_lift_verified','new_can_observation_verified'):raise SystemExit(1)


if __name__=='__main__':main()

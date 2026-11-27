"""Measured-table placement after the new horizontal-can grasp/carry.

Held pose is a rigid estimate with current hand evidence. Post-release success
uses independent actual head RGBD, not that held estimate or a planned point.
"""
from competition_paths import is_run_path
import argparse,json,math,os,signal,time,uuid
from pathlib import Path
import cv2
import numpy as np
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import LiveBackend
from apple_transfer_auto_plan import plan_placement,map_point
from horizontal_can_hand_trial import HandCanRuntime
from horizontal_can_transport_geometry import held_shape,held_silhouette,cylinder_extents
from horizontal_can_grasp_plan import forward
from can_destination_surface import measure as measure_table
from canned_place_geometry import region,approach_step,placed_expected,verify_placed,body_appearance
from canned_grasp_geometry import measure_near
from pepper_place import extension_clearance
from pregrasp_geometry import expected_horizontal_center
from inverse_table_place_plan import OFFSET,FLEX,WRIST
from task_outcome_policy import pose_matches,RELEASE_BOTTOM_GAP_M,robot_ready


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if not args.run:print(json.dumps(dict(status='horizontal_can_place_plan_only',robot_commands_sent=False)));return
    base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');carry=base/'horizontal-can-carry-20261101-01'
    if (os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or not is_run_path(args.output.resolve())):raise ValueError('isolated_domain71_D_output_required')
    prior=json.loads((carry/'result.json').read_text())
    if not prior.get('carry_destination_verified') or prior.get('release_commands_sent')!=0:
        raise ValueError('actual_destination_hold_required')
    held=json.loads((carry/'held-context.json').read_text());holding=held['holding_opening_rad']
    cfg=json.loads((carry/'config.snapshot.json').read_text());cfg['joint_feedback_max_age_s']=.6
    cfg['task']=dict(cfg['task'],object='canned_juice')
    source=base/'horizontal-can-grasp-20261101-01'
    appearance=json.loads((source/'body-appearance.json').read_text())
    reference=cv2.imread(str(base/'horizontal-can-hand-grasp-20261101-04/captures/lift-proof-0/hand.png'))
    depth_appearance=dict(source='actual_verified_can_rgbd',hue=appearance['hue'],hue_tolerance=appearance['tolerance'],
        saturation_min=70,value_min=30,verification_source='operator_confirmed_source_can_with_measured_depth')
    destination=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==cfg['destination_id'])
    root=args.output;runtime=backend=None;serial=0
    report=dict(status='starting',class_name='canned_juice',close_commands_sent=0,release_commands_sent=0,prior_close_commands_sent=1,
        placement_success_verified=False,object_remains_held=True,grasp_success_source='operator_confirmation',
        held_pose_source='rigid_kinematic_estimate_with_current_hand_RGB',task_finished_sent=False)
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    with RobotLock():
        root.mkdir(exist_ok=False);started=time.time_ns()
        with (root/'journal.jsonl').open('x') as log:
            def record(event):
                if event.get('event')=='horizontal_joint_sent' and event.get('label')=='release_once':
                    report['release_commands_sent']+=1;report['object_remains_held']=False
                log.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');log.flush();write_json(root/'result.json',report)
            def stage(name):report['stage']=name;write_json(root/'result.json',report)
            try:
                import rclpy
                rclpy.init();epoch='horizontal-can-place-'+uuid.uuid4().hex
                runtime=HandCanRuntime(root,epoch,record);backend=LiveBackend(cfg);backend.bind(root,epoch,record)
                runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
                if not pose_matches(state['joints'],prior['final_state']['joints']):raise ValueError('current_destination_hold_changed')
                if backend.room_at(state['poses']['map'])!='living_room':raise ValueError('destination_room_not_confirmed')
                write_json(root/'held-context.json',held);write_json(root/'config.snapshot.json',cfg)
                def observe(label):
                    nonlocal serial
                    last=None
                    for _ in range(3):
                        sample=runtime.sample(processing_reserve_ns=200_000_000);s=sample['snapshot'];state=runtime.state()
                        try:
                            if abs(state['joints'][5]-holding)>.03:raise ValueError('holding_opening_changed')
                            proof=held_silhouette(reference,sample['hand'],appearance)
                            shape=held_shape(s,held);plane=measure_table(sample['depth'],s,destination)
                            runtime.fresh_measurement(s);serial+=1
                            path=runtime.save(sample,f'{serial:03d}-{label}',shape)
                            return dict(measurement=shape,snapshot=s,plane=plane,state=state,capture=str(path),hand_proof=proof),sample
                        except ValueError as exc:
                            last=str(exc)
                            if last not in ('fresh_hand_image_required','fresh_hand_trial_joint_feedback_required'):raise
                    raise ValueError(last)
                stage('measure_destination_table')
                row,sample=observe('table-before-extension')
                _,flat_rotation=forward([.25,FLEX,0,WRIST,0,holding])
                offset=flat_rotation@held['center_palm_m'];axis=flat_rotation@held['axis_palm']
                vertical,horizontal=cylinder_extents(axis,held['radius_m'],held['length_m'])
                measured_held=dict(center_minus_palm_m=offset.tolist(),radius_m=horizontal,
                    holding_opening_rad=holding,bottom_below_palm_m=float(vertical-offset[2]))
                plan=plan_placement(sample['depth'],row['snapshot'],destination,row['plane'],measured_held,cfg,epoch)
                plan.update(class_name='canned_juice',height_m=held['length_m'],shape='upright_cylinder',
                    source_grasp='top_down_horizontal_can_then_standard_fold',held_pose_is_kinematic=True)
                predicted=map_point(row['snapshot'],expected_horizontal_center(plan['palm_height_m'],offset))
                clearance=extension_clearance(predicted,row['snapshot'],row['plane'],plan)
                write_json(root/'extension-clearance.json',clearance)
                if not clearance['ready']:raise ValueError('extension_not_outside_table')
                write_json(root/'placement-plan.json',plan)
                stage('prepare_horizontal_arm_at_measured_release_height')
                from table_transfer_core import prepare_release
                prepare_release(runtime,plan['palm_height_m'],holding)
                stage('full_approach_to_observed_clear_table_patch');steps=[]
                for index in range(6):
                    row,_=observe('approach');check=region(row['measurement'],row['snapshot'],plan,row['plane'])
                    if check['ready']:break
                    if index==5:raise ValueError('five_placement_corrections_exhausted')
                    if not RELEASE_BOTTOM_GAP_M[0]<=check['estimated_bottom_gap_m']<=RELEASE_BOTTOM_GAP_M[1]:
                        goal=row['state']['joints'].copy();goal[0]+=cfg['policy']['release_gap_m']-check['estimated_bottom_gap_m']
                        runtime.move_joints(goal,'correct_current_release_height');plan['palm_height_m']=goal[0]+OFFSET
                        steps.append(dict(kind='height',lift_m=goal[0]));continue
                    step=approach_step(row['measurement'],row['snapshot'],plan,row['plane']);backend.stage='approach_placement'
                    backend.adjust(f'placement-approach-{index}',row['state']['poses']['map'],step['delta_m'],
                        row['state']['joints'][:5],holding,single_placement_approach=True)
                    steps.append(dict(kind='base',delta=step['delta_m']))
                report['placement_steps']=steps
                stage('release_once')
                def release_check():
                    row,_=observe('pre-release');check=region(row['measurement'],row['snapshot'],plan,row['plane'])
                    if not check['ready']:raise ValueError('held_can_not_over_current_safe_table_support')
                    center=map_point(row['snapshot'],np.asarray(row['measurement']['center_m']))
                    plan['actual_release_xy_map_m']=center[:2].tolist()
                    plan['release_xy_source']='grasp_anchored_rigid_held_estimate_with_live_hand_RGB'
                    report['release_support']=check
                    write_json(root/'placement-plan.json',plan)
                    runtime.fresh_measurement(row['snapshot'])
                runtime.release(release_check)
                stage('withdraw_and_tuck_empty')
                runtime.raise_open();state=runtime.state();backend.stage='withdraw'
                backend.adjust('empty-retreat',state['poses']['map'],[-cfg['policy']['retreat_m'],0.,0.],state['joints'][:5],state['joints'][5])
                runtime.tuck_open()
                stage('verify_independent_actual_placed_can_depth');rows=[];errors=[]
                for index in range(9):
                    sample=runtime.sample(processing_reserve_ns=200_000_000);s=sample['snapshot']
                    if rows and s['streams']['depth']['stamp_ns']-rows[-1]['measurement']['stamp_ns']<200_000_000:
                        runtime.pump(.1);continue
                    try:
                        expected=placed_expected(s,plan)
                        measurement=measure_near(sample['depth'],s,expected,held['radius_m'],held['length_m'])
                        colour=body_appearance(sample['rgb'],sample['depth'],s,measurement,depth_appearance)
                        plane=measure_table(sample['depth'],s,destination)
                        capture=runtime.save(sample,f'placed-{index}',measurement)
                        rows.append(dict(measurement=measurement,snapshot=s,plane=plane,capture=str(capture),colour=colour))
                        if len(rows)==3:break
                    except ValueError as exc:
                        errors.append(str(exc));runtime.save(sample,f'placed-rejected-{index}')
                report['placed_measurement_errors']=errors
                verification=verify_placed(rows,plan,runtime.state()['joints'],cleanup_completed=True)
                state=runtime.state();names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
                report.update(status='horizontal_can_staged_transfer_verified',placement_success_verified=True,
                    placement_verification=verification,placed_frames=rows,final_state=state,
                    robot_ready_for_next_task=robot_ready(dict(zip(names,state['joints']))),
                    physical_transfer_verified=True,single_uninterrupted_run=False,
                    grasp_verified_by_operator=True,placed_object_independently_depth_measured=True)
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
                if runtime:
                    try:report['last_robot_state']=runtime.state()
                    except Exception:pass
            finally:
                if backend:backend.close()
                if runtime:runtime.shutdown();rclpy.shutdown()
                report['elapsed_s']=(time.time_ns()-started)/1e9
                write_json(root/'result.json',report);print(json.dumps({k:v for k,v in report.items() if k!='placed_frames'}))

if __name__=='__main__':main()

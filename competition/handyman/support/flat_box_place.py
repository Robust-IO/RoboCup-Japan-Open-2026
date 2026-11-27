"""Measured-table placement after verified flat-box grasp/carry.

Held pose is a rigid estimate with current hand evidence. Post-release success
uses independent actual head RGBD, not that held estimate or a planned point.
"""
from competition_paths import is_run_path
import argparse,json,math,os,signal,time,uuid
from pathlib import Path
import cv2
import numpy as np
from apple_transfer_automation import RobotLock,write_json
from outcome_transfer_runtime import OutcomeBackend as LiveBackend,OutcomeRuntime
from outcome_transfer_policy import snapshot as policy_snapshot,holding_feedback,hold_assessment,HoldEvidence
from apple_transfer_auto_plan import map_point
from table_approach_reachability import plan_placement,base_path
from horizontal_can_hand_trial import HandCanRuntime
from flat_box_transport import shape as held_shape,silhouette as held_silhouette,extents,from_grasp
from horizontal_can_grasp_plan import forward
from can_destination_surface import measure as measure_table
from flat_box_place_geometry import region,approach_step,placed_expected,verify_placed
from flat_box_geometry import measure_near,operator_confirmed_source
from pepper_place import extension_clearance
from pregrasp_geometry import expected_horizontal_center
from inverse_table_place_plan import OFFSET,FLEX,WRIST
from task_outcome_policy import pose_matches,RELEASE_BOTTOM_GAP_M,robot_ready
from thin_box_clearance import ThinBoxClearance
from thin_object_table_safety import require_plan
from hand_camera_ray import rotation
from head_joint_feedback import TaskHeadFeedback
from deferred_measurement_captures import DeferredMeasurementCaptures

class TaskPlaceRuntime(OutcomeRuntime,TaskHeadFeedback,HandCanRuntime):
    default_observation_profile="rgbd"


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--grasp',type=Path)
    p.add_argument('--held-run',type=Path)
    p.add_argument('--gripper-geometry',type=Path)
    p.add_argument('--destination-id')
    args=p.parse_args()
    if args.grasp and args.held_run:raise ValueError('one_held_source_required')
    if not args.run:print(json.dumps(dict(status='flat_box_place_plan_only',robot_commands_sent=False)));return
    if not args.grasp and not args.held_run:raise ValueError('current_grasp_or_carry_context_required')
    base=Path('/__HANDYMAN_PACKAGE__/runtime');carry=args.held_run or args.grasp
    if not is_run_path(carry.resolve()):raise ValueError('D_held_run_required')
    if (os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or not is_run_path(args.output.resolve())):raise ValueError('isolated_domain71_D_output_required')
    if args.grasp:
        if not is_run_path(args.grasp.resolve()):raise ValueError('D_grasp_path_required')
        held,prior,reference=from_grasp(args.grasp)
        cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text())
    else:
        prior=json.loads((carry/'result.json').read_text())
        if not prior.get('carry_destination_verified') or prior.get('release_commands_sent')!=0:raise ValueError('actual_destination_hold_required')
        held=json.loads((carry/'held-context.json').read_text())
        cfg=json.loads((carry/'config.snapshot.json').read_text())
        reference=cv2.imread(str(Path(held.get('reference_hand_capture',str(Path(held['source_run'])/'captures/lift-proof-2')))/'hand.png'))
    holding=held['holding_opening_rad'];cfg['joint_feedback_max_age_s']=.6
    if args.destination_id:cfg['destination_id']=args.destination_id
    cfg['task']=dict(cfg['task'],object='cigarette')
    appearance=held['appearance'];model=held['model']
    safety=ThinBoxClearance(args.gripper_geometry) if args.gripper_geometry else None
    if model['height_m']<=.035 and safety is None:raise ValueError('thin_object_physical_clearance_plan_required')
    destination=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==cfg['destination_id'])
    root=args.output;runtime=backend=None;serial=0
    report=dict(status='starting',class_name='cigarette',close_commands_sent=0,release_commands_sent=0,prior_close_commands_sent=1,
        placement_success_verified=False,object_remains_held=True,grasp_success_source='independent_actual_rgbd_lift_frames',
        held_pose_source='rigid_kinematic_estimate_with_current_hand_RGB',task_finished_sent=False)
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    with RobotLock():
        root.mkdir(exist_ok=False);started=time.time_ns();policy=policy_snapshot(root)
        with (root/'journal.jsonl').open('x') as log:
            def record(event):
                if event.get('event')=='horizontal_joint_sent' and event.get('label')=='release_once':
                    report['release_commands_sent']+=1;report['object_remains_held']=False
                log.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');log.flush();write_json(root/'result.json',report)
            def stage(name):report['stage']=name;write_json(root/'result.json',report)
            try:
                import rclpy
                rclpy.init();epoch='flat-box-place-'+uuid.uuid4().hex
                runtime=TaskPlaceRuntime(root,epoch,record);backend=LiveBackend(cfg);backend.bind(root,epoch,record);backend.observation_runtime=runtime
                runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
                holding_feedback(state['joints'],holding)
                from table_localization_client import localize
                stage('verify_destination_table_localization');report['localization']=localize(runtime,root,'destination')
                state=runtime.state()
                holding=float(state['joints'][5]);held['holding_opening_rad']=holding
                if backend.room_at(state['poses']['map'])!=destination['room']:raise ValueError('destination_room_not_confirmed')
                write_json(root/'held-context.json',held);write_json(root/'config.snapshot.json',cfg)
                evidence=HoldEvidence();geometry_table=destination
                def observe(label,*,check_hold=False):
                    nonlocal serial,geometry_table
                    last=None
                    for _ in range(3):
                        sample=runtime.sample(processing_reserve_ns=100_000_000);s=sample['snapshot'];state=runtime.state()
                        try:
                            holding_feedback(state['joints'],holding)
                            proof=dict(state='not_repeated',reason='verified_grasp_and_current_grip_retained')
                            if check_hold:
                                hand=runtime.sample(profile='hand')
                                proof=evidence.observe(hold_assessment(reference,hand['hand'],appearance),hand['snapshot']['streams']['hand']['stamp_ns'])
                                runtime.fresh_measurement(hand['snapshot'])
                                if proof['state']=='absent':runtime.pump(.12);continue
                            shape=held_shape(s,held);plane=measure_table(sample['depth'],s,destination)
                            geometry_table=runtime.table_geometry(sample,destination,plane)
                            runtime.fresh_measurement(s);serial+=1
                            path=runtime.save(sample,f'{serial:03d}-{label}',shape)
                            return dict(measurement=shape,snapshot=s,plane=plane,state=state,capture=str(path),hand_proof=proof),sample
                        except ValueError as exc:
                            last=str(exc)
                            if last not in ('fresh_hand_image_required','fresh_hand_trial_joint_feedback_required'):raise
                    raise ValueError(last)
                stage('measure_destination_table')
                row,sample=observe('table-before-extension',check_hold=True)
                preexisting=operator_confirmed_source(sample['rgb'],sample['depth'],row['snapshot'],destination,appearance,model,
                    operator_confirmed=False,return_all=True)
                preexisting_map=[map_point(row['snapshot'],np.asarray(m['center_m'])).tolist() for m in preexisting]
                write_json(root/'preexisting-similar-boxes.json',dict(centers_map_m=preexisting_map,source_capture=row['capture']))
                _,flat_rotation=forward([.25,FLEX,0,WRIST,0,holding])
                offset=flat_rotation@held['center_palm_m'];axes=flat_rotation@np.asarray(held['axes_palm'])
                vertical,horizontal=extents(axes,held['sizes_m'])
                measured_held=dict(center_minus_palm_m=offset.tolist(),radius_m=horizontal,
                    holding_opening_rad=holding,bottom_below_palm_m=float(vertical-offset[2]))
                plan=plan_placement(sample['depth'],row['snapshot'],geometry_table,row['plane'],measured_held,cfg,epoch,
                    minimum_shift_m=0.,held_volume=row['measurement'],outcome_policy=policy)
                plan.update(class_name='cigarette',height_m=held['sizes_m'][2],shape='flat_cuboid',
                    source_grasp='measured_flat_box_side_pinch_then_standard_fold',held_pose_is_kinematic=True,outcome_profile=policy['name'])
                predicted=map_point(row['snapshot'],expected_horizontal_center(plan['palm_height_m'],offset))
                clearance=extension_clearance(predicted,row['snapshot'],row['plane'],plan)
                write_json(root/'extension-clearance.json',clearance)
                already_extended=pose_matches(state['joints'],[state['joints'][0],FLEX,0.,WRIST,0.,holding])
                standoff_corrections=0
                while not already_extended and not clearance['ready']:
                    if standoff_corrections>=policy['maximum_corrections']:
                        raise ValueError('preextension_standoff_corrections_exhausted')
                    standoff_corrections+=1
                    retreat=clearance['minimum_clearance_m']-clearance['clearance_m']+.02
                    reach=base_path(row['state']['poses']['map'],[-retreat,0.,0.],geometry_table)
                    if not reach['ready']:raise ValueError('preextension_retreat_path_enters_table')
                    backend.stage='withdraw'
                    label='preextension-standoff' if standoff_corrections==1 else f'preextension-standoff-{standoff_corrections}'
                    backend.adjust(label,row['state']['poses']['map'],[-retreat,0.,0.],row['state']['joints'][:5],row['state']['joints'][5])
                    row,sample=observe(f'table-after-standoff-{standoff_corrections}')
                    plan=plan_placement(sample['depth'],row['snapshot'],geometry_table,row['plane'],measured_held,cfg,epoch,held_volume=row['measurement'],outcome_policy=policy)
                    plan.update(class_name='cigarette',height_m=held['sizes_m'][2],shape='flat_cuboid',held_pose_is_kinematic=True,outcome_profile=policy['name'])
                    predicted=map_point(row['snapshot'],expected_horizontal_center(plan['palm_height_m'],offset))
                    clearance=extension_clearance(predicted,row['snapshot'],row['plane'],plan)
                    record(dict(event='preextension_standoff_remeasured',correction=standoff_corrections,clearance=clearance))
                report['preextension_standoff_corrections']=standoff_corrections
                write_json(root/'extension-clearance-final.json',clearance)
                write_json(root/'placement-plan.json',plan)
                stage('prepare_horizontal_arm_at_measured_release_height')
                if safety:
                    proposed_palm=expected_horizontal_center(plan['palm_height_m'],[0.,0.,0.])
                    safe=safety.proposal(row['snapshot'],row['measurement']['center_m'],model['height_m'],proposed_palm,flat_rotation,row['plane']['plane_map'])
                    require_plan(safe,float(map_point(row['snapshot'],proposed_palm)[2]))
                    write_json(root/'thin-place-clearance-plan.json',safe)
                from table_transfer_core import prepare_release
                prepare_release(runtime,plan['palm_height_m'],holding,already_extended=already_extended)
                stage('full_approach_to_observed_clear_table_patch');steps=[]
                for index in range(policy['maximum_corrections']+2):
                    row,_=observe('approach');check=region(row['measurement'],row['snapshot'],plan,row['plane'])
                    if safety:
                        proof=safety.check(row['snapshot'],runtime.palm_pose(row['snapshot']),row['measurement']['center_m'],model['height_m'],row['plane']['plane_map'])
                        record(dict(event='thin_place_table_clearance_checked',proof=proof))
                    if check['ready']:break
                    if index==policy['maximum_corrections']+1:raise ValueError('initial_plus_five_placement_corrections_exhausted')
                    if not RELEASE_BOTTOM_GAP_M[0]<=check['estimated_bottom_gap_m']<=RELEASE_BOTTOM_GAP_M[1]:
                        goal=row['state']['joints'].copy();goal[0]+=cfg['policy']['release_gap_m']-check['estimated_bottom_gap_m']
                        if safety:
                            actual_palm=runtime.palm_pose(row['snapshot']);proposed=np.array(actual_palm['translation']);proposed[2]+=goal[0]-row['state']['joints'][0]
                            safe=safety.proposal(row['snapshot'],row['measurement']['center_m'],model['height_m'],proposed,rotation(actual_palm['quaternion']),row['plane']['plane_map'])
                            require_plan(safe,float(map_point(row['snapshot'],proposed)[2]))
                        runtime.move_joints(goal,'correct_current_release_height');plan['palm_height_m']=goal[0]+OFFSET
                        steps.append(dict(kind='height',lift_m=goal[0]));continue
                    step=approach_step(row['measurement'],row['snapshot'],plan,row['plane']);backend.stage='approach_placement'
                    reach=base_path(row['state']['poses']['map'],step['delta_m'],geometry_table)
                    if not reach['ready']:raise ValueError('current_placement_base_path_enters_table')
                    backend.adjust(f'placement-approach-{index}',row['state']['poses']['map'],step['delta_m'],
                        row['state']['joints'][:5],row['state']['joints'][5],single_placement_approach=True)
                    steps.append(dict(kind='base',delta=step['delta_m']))
                report['placement_steps']=steps
                stage('release_once')
                def release_check():
                    while True:
                        try:
                            row,_=observe('pre-release',check_hold=True);check=region(row['measurement'],row['snapshot'],plan,row['plane'])
                            if safety:
                                proof=safety.check(row['snapshot'],runtime.palm_pose(row['snapshot']),row['measurement']['center_m'],model['height_m'],row['plane']['plane_map'])
                                record(dict(event='thin_release_table_clearance_checked',proof=proof))
                            if not check['ready']:raise ValueError('held_box_not_over_current_safe_table_support')
                            center=map_point(row['snapshot'],np.asarray(row['measurement']['center_m']))
                            plan['actual_release_xy_map_m']=center[:2].tolist()
                            plan['release_xy_source']='grasp_anchored_rigid_held_estimate_with_live_hand_RGB'
                            report['release_support']=check
                            write_json(root/'placement-plan.json',plan)
                            row['snapshot']['precise_publish_check']=True
                            runtime.fresh_measurement(row['snapshot'])
                            return
                        except ValueError as exc:
                            from gate_recovery import transient_observation_error
                            if not transient_observation_error(exc):raise
                            runtime.recovery_status().waiting(str(exc))
                runtime.release(release_check)
                stage('withdraw_and_tuck_empty')
                cleanup_completed=False
                try:
                    runtime.raise_open();state=runtime.state();backend.stage='withdraw'
                    backend.adjust('empty-retreat',state['poses']['map'],[-cfg['policy']['retreat_m'],0.,0.],state['joints'][:5],state['joints'][5])
                    runtime.tuck_open();cleanup_completed=True
                except Exception as exc:
                    report['cleanup_error']=str(exc);record(dict(event='cleanup_incomplete_still_check_actual_placement',reason=str(exc)))
                stage('verify_independent_actual_placed_box_depth');rows=[];errors=[]
                for index in range(policy['verification']['placed_frames']*3):
                    sample=runtime.sample(processing_reserve_ns=100_000_000);s=sample['snapshot']
                    if rows and s['streams']['depth']['stamp_ns']-rows[-1]['measurement']['stamp_ns']<200_000_000:
                        runtime.pump(.1);continue
                    try:
                        expected=placed_expected(s,plan)
                        try:measurement=measure_near(sample['rgb'],sample['depth'],s,expected,model,appearance)
                        except ValueError:
                            # Settling may move the box while it remains correctly
                            # on this table. Reacquire over the destination surface.
                            measurement=operator_confirmed_source(sample['rgb'],sample['depth'],s,destination,appearance,model,
                                operator_confirmed=False,expected_map=plan.get('actual_release_xy_map_m',plan['release_xy_map_m']),exclude_map=preexisting_map)
                        plane=measure_table(sample['depth'],s,destination)
                        capture=runtime.save(sample,f'placed-{index}',measurement)
                        rows.append(dict(measurement=measurement,snapshot=s,plane=plane,capture=str(capture)))
                        if len(rows)>=policy['verification']['placed_frames']:break
                    except ValueError as exc:
                        errors.append(str(exc));runtime.save(sample,f'placed-rejected-{index}')
                report['placed_measurement_errors']=errors
                verification=verify_placed(rows,plan,runtime.state()['joints'],cleanup_completed=cleanup_completed)
                state=runtime.state();names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
                report.update(status='flat_box_staged_transfer_verified',placement_success_verified=True,
                    placement_verification=verification,placed_frames=rows,final_state=state,
                    robot_ready_for_next_task=robot_ready(dict(zip(names,state['joints']))),
                    physical_transfer_verified=True,single_uninterrupted_run=False,
                    grasp_verified_by_operator=Path(held['source_run']).name=='cigarette-grasp-20261101-01',grasp_verified_by_actual_rgbd=True,placed_object_independently_depth_measured=True)
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

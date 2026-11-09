"""Fixed-route ground-pepper transfer using fresh body/cap RGBD and shared motion.

Default prints a plan. Live movement requires --run and the same explicit
LayoutA simulation arguments as the existing entry. No historical run is
resumed and no failed close, release or finite trajectory is automatically replayed.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

STAGES=('preflight','navigate_source','head_view','grasp_lift','carry_destination','place_verify')


def head_view_matches(snapshot,goal=(0.,-.5)):
    joints=snapshot.get('joints',{})
    values=[joints.get(name) for name in ('head_pan_joint','head_tilt_joint')]
    return all(type(value) in (int,float) and math.isfinite(value) and abs(value-target)<=.03
        for value,target in zip(values,goal))


def validate_open_pregrasp(previous,metadata,state,boot_id):
    from task_outcome_policy import pose_matches
    if (previous.get('status')!='stopped_requires_actual_state_review'
            or previous.get('class_name')!='ground_pepper' or previous.get('stage')!='close_once'
            or previous.get('close_commands_sent')!=0 or previous.get('release_commands_sent')!=0
            or previous.get('grasp_success_verified') is not False or metadata.get('boot_id')!=boot_id
            or not pose_matches(state['joints'],previous['last_fresh_robot_state']['joints'])
            or abs(state['joints'][5]-.65)>.005):
        raise ValueError('reviewed_open_pregrasp_and_zero_previous_close_required')


def validate_destination_hold(previous,metadata,held,state,boot_id):
    from task_outcome_policy import pose_matches
    holding=held.get('holding_opening_rad',-1)
    if (previous.get('status')!='stopped_requires_actual_state_review'
            or previous.get('class_name')!='ground_pepper'
            or previous.get('stage')!='measure_destination_and_plan'
            or previous.get('grasp_success_verified') is not True or previous.get('release_commands_sent')!=0
            or 'carry_destination' not in previous.get('confirmed_stages',[])
            or held.get('class_name')!='ground_pepper' or held.get('grasp_lift_verified') is not True
            or not .02<holding<=.62 or metadata.get('boot_id')!=boot_id or held.get('boot_id')!=boot_id
            or held.get('run_id')!=metadata.get('run_id') or held.get('run_root')!=metadata.get('run_root')
            or not pose_matches(state['joints'],[0.,0.,0.,-math.pi/2,0.,holding])):
        raise ValueError('verified_unreleased_ground_pepper_destination_hold_required')


def validate_lifted_hold(previous,metadata,held,state,boot_id):
    from task_outcome_policy import pose_matches
    if (previous.get('status')!='stopped_requires_actual_state_review'
            or previous.get('class_name')!='ground_pepper' or previous.get('stage')!='verify_hold_before_retraction'
            or previous.get('grasp_success_verified') is not True or previous.get('close_commands_sent')!=1
            or previous.get('release_commands_sent')!=0 or 'grasp_lift' not in previous.get('confirmed_stages',[])
            or metadata.get('boot_id')!=boot_id or held.get('boot_id')!=boot_id
            or held.get('run_id')!=metadata.get('run_id') or held.get('run_root')!=metadata.get('run_root')
            or held.get('class_name')!='ground_pepper' or held.get('grasp_lift_verified') is not True
            or not .02<held.get('holding_opening_rad',-1)<=.62
            or not pose_matches(state['joints'],previous['last_fresh_robot_state']['joints'])
            or not pose_matches(state['joints'],[state['joints'][0],-1.7,0.,.1292036732051034,0.,held['holding_opening_rad']])):
        raise ValueError('verified_horizontal_lifted_pepper_hold_required')


def validate_horizontal_placement_hold(previous,metadata,origin,held,plan,state,boot_id):
    from task_outcome_policy import pose_matches
    holding=held.get('holding_opening_rad',-1)
    if (previous.get('status')!='stopped_requires_actual_state_review'
            or previous.get('class_name')!='ground_pepper' or previous.get('stage')!='approach_measured_release_region'
            or previous.get('grasp_success_verified') is not True or previous.get('release_commands_sent')!=0
            or 'carry_destination' not in previous.get('confirmed_stages',[])
            or metadata.get('boot_id')!=boot_id or origin.get('boot_id')!=boot_id or held.get('boot_id')!=boot_id
            or held.get('run_id')!=origin.get('run_id') or held.get('run_root')!=origin.get('run_root')
            or previous.get('original_grasp_run',previous.get('predecessor_run',metadata.get('run_root')))!=held.get('run_root')
            or held.get('class_name')!='ground_pepper' or held.get('grasp_lift_verified') is not True
            or plan.get('class_name')!='ground_pepper' or not .02<holding<=.62
            or not pose_matches(state['joints'],previous['last_fresh_robot_state']['joints'])
            or not pose_matches(state['joints'],[state['joints'][0],-1.7,0.,.1292036732051034,0.,holding])):
        raise ValueError('verified_unreleased_horizontal_placement_hold_required')


def validate_navigation_hold(previous,metadata,held,state,boot_id,navigation,route):
    from task_outcome_policy import pose_matches
    holding=held.get('holding_opening_rad',-1)
    if (previous.get('status')!='stopped_requires_actual_state_review'
            or previous.get('class_name')!='ground_pepper' or previous.get('stage')!='navigate_destination'
            or previous.get('grasp_success_verified') is not True or previous.get('close_commands_sent')!=1
            or previous.get('release_commands_sent')!=0 or 'grasp_lift' not in previous.get('confirmed_stages',[])
            or metadata.get('boot_id')!=boot_id or held.get('boot_id')!=boot_id
            or held.get('run_id')!=metadata.get('run_id') or held.get('run_root')!=metadata.get('run_root')
            or held.get('class_name')!='ground_pepper' or held.get('grasp_lift_verified') is not True
            or not .02<holding<=.62 or not pose_matches(state['joints'],[0.,0.,0.,-math.pi/2,0.,holding])
            or navigation.get('status')!='failed' or navigation.get('nav_reset') is not True
            or navigation.get('speed_parameters_restored') is not True
            or not navigation.get('error','').startswith('stale_navigation_feedback:')
            or navigation.get('target')!=route[-1]):
        raise ValueError('verified_unreleased_navigation_hold_required')
    reached=navigation.get('reached',[])
    if len(reached)>=len(route) or any(row.get('status')!=4 or row.get('index')!=i or row.get('target')!=route[i]
            for i,row in enumerate(reached)):
        raise ValueError('completed_waypoint_prefix_required')
    return route[len(reached):-1]


def validate_released_placement(previous,metadata,held,plan,state,boot_id,*,verification_only=False):
    from task_outcome_policy import pose_matches,hand_is_open,robot_ready
    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    pose_ok=(robot_ready(dict(zip(names,state['joints']))) if verification_only else
        pose_matches(state['joints'][:5]+[0.],previous['last_fresh_robot_state']['joints'][:5]+[0.]))
    if (previous.get('status')!='stopped_requires_actual_state_review'
            or previous.get('class_name')!='ground_pepper' or previous.get('stage')!='verify_actual_placement'
            or previous.get('release_commands_sent')!=1 or previous.get('hand_released') is not True
            or previous.get('grasp_success_verified') is not True or metadata.get('boot_id')!=boot_id
            or held.get('boot_id')!=boot_id or held.get('class_name')!='ground_pepper'
            or plan.get('class_name')!='ground_pepper' or 'actual_release_xy_map_m' not in plan
            or not hand_is_open(state['joints'][5])
            or not pose_ok):
        raise ValueError('one_previous_release_and_current_open_arm_required')


def confirmed_objects(detections,name):
    from can_identity_selection import overlap
    candidates=[]
    for d in detections:
        box=d.get('box',[]);score=d.get('confidence')
        if (d.get('name')!=name or type(score) not in (float,int) or not math.isfinite(score)
                or not .8<=score<=1 or len(box)!=4 or not all(math.isfinite(x) for x in box)
                or box[2]<=box[0] or box[3]<=box[1]):continue
        candidates.append(d)
    kept=[]
    for d in sorted(candidates,key=lambda v:v['confidence'],reverse=True):
        if not any(overlap(d['box'],p['box'])>=.85 for p in kept):kept.append(d)
    return kept


def confirmed_peppers(detections):
    return confirmed_objects(detections,'ground_pepper')


def run_live(output,task_id,*,observe_only=False,grasp_only=False,continue_open_pregrasp=None,
        continue_destination_hold=None,continue_released_placement=None,verify_released_placement=None,task_config=None,
        continue_navigation_hold=None,continue_lifted_hold=None,object_name='ground_pepper',navigation_profile=None):
    import numpy as np
    import yaml
    import rclpy
    from apple_transfer_automation import RobotLock,check_inputs,write_json
    from apple_transfer_auto_backend import LiveBackend
    from apple_transfer_auto_plan import footprint,map_point,map_pose
    from apple_worker_client import Worker
    from canned_grasp_geometry import fresh,transform_point
    from canned_carry_geometry import current_images_and_feedback,tucked_holding
    from inverse_table_place_plan import OFFSET,FLEX,WRIST
    from manipulation_profiles import object_profile
    from pepper_geometry import estimate_source,measure_near,lift_follow
    from pepper_runtime import PepperRuntime
    from pepper_hold import palm_points,expected_centers,current_depth,rgb_continuation,current_cap_depth
    from pepper_place import make_plan,approach_step,region,expected_placed,verify_placed,extension_clearance
    from pepper_tasks import load_task,configured_transfer,task_digest,validate_predecessor_task
    from pepper_source_observation import source_identity_evidence
    from can_destination_surface import measure as measure_table
    if object_name=='toy_penguin':
        from penguin_hand_hold import create_reference,verify_hand,contact_context,verify_lift as hand_lift,estimate_held,hold_evidence as hand_hold_evidence
        from penguin_source_identity import make_seed,proposal as seed_proposal,confirm as confirm_seed,source_table_matches
        from penguin_geometry import estimate_source,measure_near,lift_follow
        from penguin_runtime import PenguinRuntime as PepperRuntime
        from penguin_hold import palm_points,expected_centers,current_depth,rgb_continuation,current_cap_depth
        from penguin_place import make_plan,approach_step,region,expected_placed,verify_placed,extension_clearance
        from penguin_tasks import load_task,configured_transfer,task_digest,validate_predecessor_task
    elif object_name!='ground_pepper':raise ValueError('unsupported_round_transfer_object')
    from task_outcome_policy import robot_ready
    import sys
    code=Path(__file__).parent;sys.path.insert(0,str(code/'scripts'))
    from scripts.search_room_gate import inside
    baseline_cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text());object_profile(object_name)
    task=load_task(task_config)
    environment=yaml.safe_load(Path(baseline_cfg['environment']).read_text())
    tables=json.loads(Path(baseline_cfg['semantic_map']).read_text())['destinations']
    cfg=configured_transfer(baseline_cfg,task,environment,tables)
    if navigation_profile is not None:
        from motion_speed import load_profile
        load_profile(navigation_profile);cfg['navigation_motion_profile']=navigation_profile
    source_room,destination_room=cfg['source_room'],cfg['destination_room']
    head_goal=cfg['source_head_pose']
    source=next(t for t in tables if t['id']==cfg['source_table_id'])
    destination=next(t for t in tables if t['id']==cfg['destination_id'])
    root=Path(output);runtime=None;backend=None;worker=None;serial=0;held=None;plan=None;reuse_horizontal_placement=False
    report=dict(status='starting',mode='live',class_name=object_name,task_id=task_id,
        physical_transfer_verified=False,grasp_success_verified=False,placement_success_verified=False,
        robot_commands_sent=False,
        confirmed_stages=[],close_commands_sent=0,release_commands_sent=0,approach_commands_sent=0,automatic_action_retry=False,
        historical_checkpoint_used=False,navigation_parameters_changed=False,model_changed=False,task_finished_sent=False)
    report['continued_from_open_pregrasp']=continue_open_pregrasp is not None
    report['continued_from_destination_hold']=continue_destination_hold is not None
    report['continued_from_navigation_hold']=continue_navigation_hold is not None
    report['continued_from_lifted_hold']=continue_lifted_hold is not None
    report['continued_from_released_placement']=continue_released_placement is not None
    report['placement_verification_only']=verify_released_placement is not None
    report['automatic_action_retry']=bool(cfg.get('navigation_feedback_recovery',False))
    report['automatic_manipulation_retry']=False
    report['navigation_retry_scope']='unfinished_waypoint_after_confirmed_cancellation_and_new_feedback'
    report['joint_feedback_max_age_s']=.6
    report['rgbd_image_max_age_s']=.4
    report['navigation_motion_profile']=cfg.get('navigation_motion_profile',cfg['motion_profile'])
    report['configured_task']=task
    report['task_sha256']=task_digest(task)
    report['fresh_unmoved_unity_restart_verified']=False
    with RobotLock():
        root.mkdir(parents=True,exist_ok=False);started=time.time_ns()
        meta=dict(version=1,run_id=task_id,object=object_name,run_root=str(root),mode='live',started_ns=started,
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),task_sha256=task_digest(task))
        write_json(root/'run-meta.json',meta);write_json(root/'config.snapshot.json',cfg)
        with (root/'journal.jsonl').open('x') as journal:
            def save():write_json(root/'result.json',report)
            def record(event):
                if event.get('event') in ('finite_can_approach_sent','can_closure_sent','pepper_joint_command_sent','pepper_head_command_sent'):
                    report['robot_commands_sent']=True
                if event.get('event')=='finite_can_approach_sent':report['approach_commands_sent']+=1;save()
                if event.get('event')=='can_closure_sent':report['close_commands_sent']+=1;save()
                if event.get('event')=='pepper_joint_command_sent' and event.get('label')=='release_once':
                    report['release_commands_sent']+=1;save()
                row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row,allow_nan=False)+'\n');journal.flush()
                print(json.dumps(row,allow_nan=False),flush=True)
            def stage(name):
                report['stage']=name;save();record(dict(event='stage',stage=name))
            def folder(name):
                path=root/name;path.mkdir(exist_ok=False);runtime.root=path
            def measured(label,expected_body,expected_cap,model,*,motion_anchor=None):
                nonlocal serial
                last='no_current_pepper_surface'
                for _ in range(5):
                    sample=runtime.sample(processing_reserve_ns=200_000_000);s=sample['snapshot'];serial+=1
                    try:
                        motion_proof=None
                        eb,ec=expected_body(s),expected_cap(s)
                        if motion_anchor is not None:
                            try:
                                corrected=motion_anchor.predict(sample,eb,ec)
                                if corrected is not None:
                                    eb,ec,motion_proof=corrected
                                    record(dict(event='pepper_static_background_search_prediction',**motion_proof))
                            except ValueError as exc:
                                record(dict(event='pepper_visual_motion_unavailable_keep_original_roi',reason=str(exc)))
                        if hasattr(runtime,'recenter_target_view') and runtime.recenter_target_view(eb,ec,s):continue
                        if object_name=='ground_pepper' and report['close_commands_sent']==0:
                            # Table-supported source prior selects an ROI only.
                            # Current body/cap fits still determine actual tilt.
                            ec=np.asarray(eb)+[0.,0.,model['body_cap_distance_m']]
                        options=dict(open_pregrasp=True) if object_name=='toy_penguin' and report['close_commands_sent']==0 else {}
                        m=measure_near(sample['rgb'],sample['depth'],s,eb,ec,model,**options)
                        runtime.fresh_measurement(s)
                        if hasattr(runtime,'accept_geometry'):runtime.accept_geometry(m,s)
                        if object_name=='ground_pepper' and motion_proof is not None:
                            runtime.accept_visual_motion(motion_proof,s)
                        capture=runtime.save(sample,f'{serial:03d}-{label}',m)
                        return sample,m,capture
                    except ValueError as exc:
                        last=str(exc);capture=runtime.save(sample,f'{serial:03d}-{label}-rejected')
                        record(dict(event='pepper_new_frame_rejected_no_action_replayed',reason=last,capture=str(capture)))
                raise ValueError('pepper_current_geometry_not_observed:'+last)
            def hold_frames(label,*,allow_rgb=False,allow_cap=False):
                nonlocal serial
                rows=[];deadline=time.monotonic()+8;last='no_current_held_pepper'
                while len(rows)<3 and time.monotonic()<deadline:
                    sample=runtime.sample(processing_reserve_ns=200_000_000);s=sample['snapshot'];palm=runtime.palm_pose(s)
                    try:
                        try:
                            if object_name=='toy_penguin' and held.get('hand_face_reference'):
                                evidence=hand_hold_evidence(sample,palm,held,now_ns=runtime.node.get_clock().now().nanoseconds)
                            else:evidence=current_depth(sample['rgb'],sample['depth'],s,palm,held)
                        except ValueError:
                            if object_name=='toy_penguin' and held.get('hand_face_reference'):raise
                            if allow_cap:evidence=current_cap_depth(sample['rgb'],sample['depth'],s,palm,held)
                            elif not allow_rgb:raise
                            else:evidence=rgb_continuation(sample['rgb'],sample['depth'],s,palm,held)
                        timing=current_images_and_feedback(s,runtime.state(),runtime.node.get_clock().now().nanoseconds,runtime.epoch,
                            joint_max_age_ns=600_000_000)
                        if rows and s['streams']['depth']['stamp_ns']-rows[-1]['stamp_ns']<200_000_000:continue
                        serial+=1;capture=runtime.save(sample,f'{serial:03d}-{label}',evidence.get('measurement'))
                        row=dict(evidence,timing=timing,capture=str(capture),stamp_ns=s['streams']['depth']['stamp_ns'])
                        rows.append(row)
                    except ValueError as exc:last=str(exc)
                    except RuntimeError as exc:
                        if str(exc) not in ('feedback_wait_expired:stale_joints','feedback_wait_expired:stale_base_tf',
                                'feedback_wait_expired','stale_joints','stale_base_tf'):raise
                        last=str(exc)
                        record(dict(event='temporary_current_hold_feedback_wait_without_motion',reason=last))
                if len(rows)!=3:raise ValueError('pepper_current_hold_not_verified:'+label+':'+last)
                write_json(runtime.root/(label+'.json'),dict(rows=rows,current_held_verified=True));return rows
            try:
                write_json(root/'input-audit.json',dict(check_inputs(baseline_cfg),configured_task=task,task_sha256=task_digest(task)))
                backend=LiveBackend(cfg);backend.bind(root,task_id,record);backend.stage='preflight'
                if continue_released_placement is not None or verify_released_placement is not None:
                    stage('preflight')
                    predecessor=Path(continue_released_placement or verify_released_placement)
                    previous=json.loads((predecessor/'result.json').read_text())
                    previous_meta=json.loads((predecessor/'run-meta.json').read_text())
                    validate_predecessor_task(cfg,json.loads((predecessor/'config.snapshot.json').read_text()))
                    held=json.loads((predecessor/'held-context.json').read_text())
                    plan=json.loads((predecessor/'place/placement-plan.json').read_text())
                    rclpy.init();runtime=PepperRuntime(root,task_id+'-rgbd',record)
                    runtime.sample();state=runtime.state();runtime.ownership()
                    validate_released_placement(previous,previous_meta,held,plan,state,meta['boot_id'],
                        verification_only=verify_released_placement is not None)
                    if (backend.room_at(state['poses']['map'])!=destination_room or plan['destination_id']!=cfg['destination_id']):
                        raise ValueError('released_pepper_destination_context_changed')
                    model=held['model'];holding=held['holding_opening_rad']
                    report.update(grasp_success_verified=True,hand_released=True,predecessor_run=str(predecessor),
                        previous_release_commands_sent=1,historical_checkpoint_used=True,navigation_performed_in_this_run=False)
                    write_json(root/'preflight.json',dict(current_state=state,ready=True,previous_release_verified=True,
                        actions_replayed=False,object_is_not_treated_as_still_held=True))
                    report['confirmed_stages'].append('preflight')
                elif continue_destination_hold is not None or continue_navigation_hold is not None or continue_lifted_hold is not None:
                    stage('preflight')
                    predecessor=Path(continue_destination_hold or continue_navigation_hold or continue_lifted_hold)
                    previous=json.loads((predecessor/'result.json').read_text())
                    previous_meta=json.loads((predecessor/'run-meta.json').read_text())
                    validate_predecessor_task(cfg,json.loads((predecessor/'config.snapshot.json').read_text()))
                    held=json.loads((predecessor/'held-context.json').read_text())
                    rclpy.init();runtime=PepperRuntime(root,task_id+'-rgbd',record)
                    runtime.sample();state=runtime.state();runtime.ownership()
                    if continue_lifted_hold is not None:
                        validate_lifted_hold(previous,previous_meta,held,state,meta['boot_id'])
                        if backend.room_at(state['poses']['map'])!=source_room:raise ValueError('lifted_hold_not_in_source_room')
                        remaining=backend.route(source_room,destination_room)
                    elif continue_navigation_hold is not None:
                        navigation=json.loads((predecessor/'destination-navigation.json').read_text())
                        full_route=backend.route(source_room,destination_room)+[cfg['destination_staging_goal']]
                        remaining=validate_navigation_hold(previous,previous_meta,held,state,meta['boot_id'],navigation,full_route)
                    else:
                        if previous.get('stage')=='approach_measured_release_region':
                            origin_path=Path(held['run_root'])
                            if not origin_path.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs')):
                                raise ValueError('original_grasp_context_outside_D_drive')
                            origin=json.loads((origin_path/'run-meta.json').read_text())
                            plan=json.loads((predecessor/'place/placement-plan.json').read_text())
                            validate_horizontal_placement_hold(previous,previous_meta,origin,held,plan,state,meta['boot_id'])
                            if plan['destination_id']!=cfg['destination_id']:raise ValueError('placement_destination_changed')
                            reuse_horizontal_placement=True
                        else:validate_destination_hold(previous,previous_meta,held,state,meta['boot_id'])
                        if backend.room_at(state['poses']['map'])!=destination_room:
                            raise ValueError('continued_hold_not_at_destination_room')
                    holding=held['holding_opening_rad'];model=held['model']
                    report.update(grasp_success_verified=True,held_opening_rad=holding,
                        predecessor_run=str(predecessor),previous_grasp_lift_and_carry_verified=continue_destination_hold is not None,
                        previous_close_commands_sent=previous['close_commands_sent'],original_grasp_run=held['run_root'],historical_checkpoint_used=True,
                        navigation_performed_in_this_run=False)
                    write_json(root/'held-context.json',held)
                    write_json(root/'preflight.json',dict(current_state=state,ready=True,
                        prior_grasp_verified=True,prior_release_commands=0,actions_replayed=False))
                    report['confirmed_stages'].append('preflight')
                    folder('carry');stage('verify_current_held_pepper')
                    hold_frames('current-held-before-continuation',allow_rgb=True,allow_cap=continue_lifted_hold is not None)
                    if continue_lifted_hold is not None:
                        stage('retract_preserving_grip');runtime.retract_holding(holding)
                        hold_frames('after-retract',allow_rgb=True)
                    if continue_navigation_hold is not None or continue_lifted_hold is not None:
                        stage('navigate_destination');backend.stage='navigate_destination'
                        backend.navigate(source_room,destination_room,cfg['destination_staging_goal'],holding,
                            'destination-navigation',remaining_waypoints=remaining)
                        report['navigation_performed_in_this_run']=True
                        hold_frames('current-destination-held',allow_rgb=True)
                        report['confirmed_stages'].append('carry_destination')
                    else:report['confirmed_stages'].append('destination_hold')
                    save()
                else:
                    stage('preflight');preflight=root/'preflight.json'
                    if continue_open_pregrasp is not None:
                        predecessor=Path(continue_open_pregrasp)
                        previous=json.loads((predecessor/'result.json').read_text());previous_meta=json.loads((predecessor/'run-meta.json').read_text())
                        validate_predecessor_task(cfg,json.loads((predecessor/'config.snapshot.json').read_text()))
                        rclpy.init();runtime=PepperRuntime(root,task_id+'-rgbd',record)
                        runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
                        validate_open_pregrasp(previous,previous_meta,state,meta['boot_id'])
                        backend.start_room=backend.room_at(state['poses']['map']);backend.initial_opening=state['joints'][5]
                        if backend.start_room!=source_room:raise ValueError('open_pregrasp_must_be_in_source_room')
                        write_json(preflight,dict(ready=True,current_state=state,previous_run=str(predecessor),
                            previous_close_commands=0,actions_replayed=False,source_identity_must_be_reacquired=True))
                    else:
                        ready=backend.command([code/'inspect_transfer_ready.py','--output',preflight,'--efficient-transfer',
                            '--joint-feedback-max-age-s','.6'],preflight,'ready',True,25)
                        backend.start_room=backend.room_at(ready['base_map_pose']);backend.initial_opening=ready['joints']['hand_motor_joint']
                        report['initial_base_map_pose']=ready['base_map_pose']
                        report['start_kind']='fresh_empty_tucked_current_pose'
                    if backend.start_room not in environment['rooms']:raise ValueError('pepper_start_outside_reviewed_route')
                    report['confirmed_stages'].append('preflight')
                    if not observe_only and continue_open_pregrasp is None:
                        stage('navigate_source');backend.stage='navigate_source'
                        backend.navigate(backend.start_room,source_room,cfg['source_staging_goal'],backend.initial_opening,'source-navigation')
                        report['robot_commands_sent']=True
                        report['confirmed_stages'].append('navigate_source')
                    if runtime is None:rclpy.init();runtime=PepperRuntime(root,task_id+'-rgbd',record)
                    if not observe_only:
                        stage('head_view');backend.stage='head_view'
                        current_view=runtime.sample(processing_reserve_ns=0)
                        if head_view_matches(current_view['snapshot'],head_goal):
                            capture=runtime.save(current_view,'already-facing-source')
                            record(dict(event='current_head_view_confirmed_without_redundant_motion',capture=str(capture)))
                        elif continue_open_pregrasp is not None:raise ValueError('open_pregrasp_head_pose_changed')
                        else:runtime.face_source(head_goal)
                        report['confirmed_stages'].append('head_view')
                    folder('grasp');stage('new_'+object_name+'_rgbd')
                    worker=Worker(cfg['model_python'],cfg['registry'],runtime.root/'model-worker.log',imgsz=640)
                    registry=json.loads(Path(cfg['registry']).read_text())
                    if worker.ready.get('model_sha256')!=registry['sha256']:raise ValueError('pepper_current_model_mismatch')
                    identities=[];source_seed=None;table_center,table_half=footprint(source)
                    for index in range(9):
                        sample=runtime.sample(processing_reserve_ns=200_000_000);capture=runtime.save(sample,f'identity-{index:02d}')
                        raw=worker.infer(capture,pump=lambda:runtime.pump(0.));write_json(capture/'raw-detector-result.json',raw)
                        targets=confirmed_objects(raw.get('detections',[]),object_name)
                        if len(targets)!=1 and source_seed is not None:
                            proposed=seed_proposal(source_seed,sample['snapshot'])
                            if proposed is not None:targets=[proposed]
                        if len(targets)!=1:
                            if (hasattr(runtime,'improve_unconfirmed_source_view')
                                    and runtime.improve_unconfirmed_source_view(raw.get('detections',[]),sample['snapshot'])):
                                identities=[]
                            continue
                        if object_name=='toy_penguin' and (targets[0]['box'][0]<=2 or targets[0]['box'][2]>=sample['depth'].shape[1]-2):
                            try:
                                seed_plane=measure_table(sample['depth'],sample['snapshot'],source)
                                seed_geometry=estimate_source(sample['rgb'],sample['depth'],sample['snapshot'],targets[0],
                                    measured_support=seed_plane,allow_clipped_seed=True)
                                source_seed=make_seed(seed_geometry,sample['snapshot'],targets[0],capture)
                                write_json(runtime.root/'source-identity-seed.json',source_seed)
                                record(dict(event='penguin_high_confidence_identity_seed_observed',capture=str(capture),
                                    confidence=source_seed['confidence'],motion_authorized=False))
                            except ValueError as exc:record(dict(event='penguin_partial_identity_seed_not_available',reason=str(exc)))
                        if (hasattr(runtime,'recenter_clipped_detection')
                                and runtime.recenter_clipped_detection(targets[0],sample['snapshot'])):
                            identities=[];continue
                        try:
                            source_plane=measure_table(sample['depth'],sample['snapshot'],source)
                            m=estimate_source(sample['rgb'],sample['depth'],sample['snapshot'],targets[0],measured_support=source_plane)
                            if targets[0].get('fresh_model_detection') is False:
                                m['identity_continuity']=confirm_seed(source_seed,m,sample['snapshot'])
                            elif object_name=='toy_penguin' and source_seed is None:
                                source_seed=make_seed(m,sample['snapshot'],targets[0],capture)
                                write_json(runtime.root/'source-identity-seed.json',source_seed)
                            current=runtime.state()
                            latest_joints=runtime.received['joints']
                            values=dict(zip(latest_joints.name,latest_joints.position))
                            current_head=[values[k] for k in ('head_pan_joint','head_tilt_joint')]
                            timing=source_identity_evidence(sample['snapshot'],current,current_head,
                                runtime.node.get_clock().now().nanoseconds,runtime.epoch)
                        except ValueError as exc:
                            now=runtime.node.get_clock().now().nanoseconds
                            record(dict(event='source_observation_rejected_without_motion',reason=str(exc),capture=str(capture),
                                oldest_image_age_s=(now-min(sample['snapshot']['streams'][k]['stamp_ns'] for k in ('rgb','depth')))/1e9))
                            report['last_source_observation_rejection']=str(exc);save();continue
                        mapped=map_point(sample['snapshot'],np.asarray(m['center_m']))
                        source_associated=(source_table_matches(mapped,table_center,table_half,m['body_radius_m'])
                            if object_name=='toy_penguin' else bool(np.all(abs(mapped[:2]-table_center)<=table_half)))
                        if (not inside(*mapped[:2],environment['rooms'][source_room]['region'])
                                or not source_associated):
                            raise ValueError(object_name+'_not_on_requested_source_table')
                        identities.append(dict(capture=str(capture),detection=targets[0],measurement=m,timing=timing,
                            center_odom_m=transform_point(sample['snapshot'],m['center_m']).tolist()))
                        if len(identities)==3:break
                    worker.close();worker=None
                    if len(identities)!=3:raise ValueError('three_new_'+object_name+'_observations_required')
                    centers=np.asarray([r['center_odom_m'] for r in identities])
                    if np.max(np.linalg.norm(centers-centers.mean(0),axis=1))>.015:raise ValueError('pepper_source_not_repeatable')
                    report['source_observations']=identities;model=m
                    write_json(runtime.root/'source-reference.json',model)
                    if observe_only:
                        report.update(status='new_'+object_name+'_observation_verified',robot_commands_sent=False);return report
                    body_odom=transform_point(sample['snapshot'],m['center_m']);cap_odom=transform_point(sample['snapshot'],m['cap_center_m'])
                    motion_anchor=None
                    if object_name=='ground_pepper':
                        from pepper_visual_motion import NearAnchor
                        motion_anchor=NearAnchor(sample,m)
                    def observe():
                        nonlocal body_odom,cap_odom,motion_anchor
                        item,measurement,_=measured('near',lambda s:transform_point(s,body_odom,inverse=True),
                            lambda s:transform_point(s,cap_odom,inverse=True),model,motion_anchor=motion_anchor)
                        if object_name=='ground_pepper':motion_anchor=NearAnchor(item,measurement)
                        if object_name=='ground_pepper' or object_name=='toy_penguin' and report['close_commands_sent']==0:
                            body_odom=transform_point(item['snapshot'],measurement['center_m'])
                            cap_odom=transform_point(item['snapshot'],measurement['cap_center_m'])
                        return item,measurement
                    grasp_point=np.asarray(m['grasp_center_m'] if object_name=='toy_penguin' else m['center_m'])
                    grasp_above=float(grasp_point[2]-np.r_[grasp_point[:2],1.]@m['support_plane_base']) if object_name=='toy_penguin' else model['body_above_support_m']
                    lift=float(grasp_point[2]-OFFSET)
                    if not .07<=lift<=.343:raise ValueError('pepper_measured_body_height_outside_reviewed_arm_range')
                    stage('prepare_horizontal_at_wide_body')
                    if continue_open_pregrasp is None:arm=runtime.prepare(lift,grasp_above)
                    else:
                        from task_outcome_policy import pose_matches
                        state=runtime.state()
                        if not pose_matches(state['joints'],[lift,FLEX,0.,WRIST,0.,.65]):
                            raise ValueError('current_horizontal_arm_does_not_match_new_measured_target_height')
                        runtime.vertical_alignment_tolerance_m=min(.006,model['body_above_support_m']-.0175-.004)
                        if runtime.vertical_alignment_tolerance_m<=0:raise ValueError('pepper_body_too_low_for_finger_clearance')
                        arm=state['joints'][:5]
                        record(dict(event='current_empty_horizontal_arm_confirmed_without_replaying_preparation'))
                    stage('align_and_approach_wide_body');approaches=[]
                    for index in range(6):
                        sample,m=observe()
                        if runtime.is_aligned(m['center_m'],sample['snapshot']):break
                        for attempt in range(5):
                            sent_before=report['approach_commands_sent']
                            try:
                                approaches.append(runtime.approach(observe,arm,index));break
                            except ValueError as exc:
                                if (str(exc) not in ('fresh_carry_images_required','fresh_current_carry_joint_feedback_required')
                                        or report['approach_commands_sent']!=sent_before or attempt==4):raise
                                record(dict(event='expired_preapproach_image_reacquire_without_motion',attempt=attempt,reason=str(exc)))
                        report['approaches']=approaches;save()
                    stage('close_once')
                    for attempt in range(5):
                        try:
                            close,before_sample,before=runtime.close_gripper(observe,arm);break
                        except ValueError as exc:
                            if (str(exc) not in ('fresh_carry_images_required','fresh_current_carry_joint_feedback_required')
                                    or report['close_commands_sent'] or attempt==4):raise
                            record(dict(event='expired_preclose_image_reacquire_without_close',attempt=attempt,reason=str(exc)))
                    holding=close['closure_evidence']['observed_opening_rad'];report['close']=close
                    if not .02<holding<=.62:raise ValueError('pepper_contact_not_verified_no_lift')
                    hand_contact=None
                    if object_name=='toy_penguin':
                        post_sample,post=before_sample,before
                        record(dict(event='genuine_preclose_rgbd_reused_as_lift_baseline_after_contact',
                            source_capture=close['before_capture'],redundant_postclose_depth_gate=False))
                        try:
                            hand_ref=create_reference(before_sample['hand'],before_sample['snapshot'],before,
                                runtime.root/'hand-face-reference.png',model=model)
                            hand_ref['source_capture']=close['before_capture']
                            closed=runtime.sample();closed_evidence=verify_hand(closed['hand'],closed['snapshot'],hand_ref,holding,
                                now_ns=runtime.node.get_clock().now().nanoseconds)
                            hand_contact=contact_context(model,before,before_sample['snapshot'],hand_ref,holding)
                            capture=runtime.save(closed,'postclose-hand-face-confirmed')
                            write_json(runtime.root/'hand-face-reference.json',hand_ref)
                            record(dict(event='current_postclose_hand_face_verified',capture=str(capture),evidence=closed_evidence))
                        except ValueError as exc:
                            record(dict(event='penguin_hand_reference_unavailable_use_depth_lift_proof',reason=str(exc)))
                    else:
                        post_sample,post=observe();runtime.save(post_sample,'postclose-before-lift',post)
                    stage('verify_lift');runtime.lift_holding(holding)
                    c0=transform_point(post_sample['snapshot'],post['center_m']);c1=transform_point(post_sample['snapshot'],post['cap_center_m'])
                    p0=transform_point(post_sample['snapshot'],post_sample['snapshot']['palm_base_at_depth'])
                    def lifted_expected(s,original):
                        movement=transform_point(s,s['palm_base_at_depth'])-p0
                        return transform_point(s,original+movement,inverse=True)
                    lifted=[]
                    for _ in range(9):
                        if hand_contact is not None:
                            sample=runtime.sample();s=sample['snapshot']
                            evidence=verify_hand(sample['hand'],s,hand_contact['hand_face_reference'],holding,
                                now_ns=runtime.node.get_clock().now().nanoseconds)
                            proof=hand_lift(post_sample['snapshot'],s,evidence,holding)
                            if lifted and proof['stamp_ns']-lifted[-1]['proof']['stamp_ns']<200_000_000:
                                runtime.pump(.1);continue
                            m=estimate_held(runtime.palm_pose(s),s,dict(hand_contact,grasp_lift_verified=True),evidence)
                            runtime.fresh_measurement(s);serial+=1
                            capture=runtime.save(sample,f'{serial:03d}-after-lift-hand-rigidity',m)
                        else:
                            sample,m,capture=measured('after-lift',lambda s:lifted_expected(s,c0),lambda s:lifted_expected(s,c1),model)
                            proof=lift_follow(post,post_sample['snapshot'],m,sample['snapshot'],holding)
                        if lifted and proof['stamp_ns']-lifted[-1]['proof']['stamp_ns']<200_000_000:continue
                        bp,cp=palm_points(m['center_m'],m['cap_center_m'],runtime.palm_pose(sample['snapshot']))
                        lifted.append(dict(capture=str(capture),measurement=m,proof=proof,body_palm_m=bp,cap_palm_m=cp))
                        if len(lifted)==3:break
                    if len(lifted)!=3:raise ValueError('three_current_pepper_lift_frames_required')
                    held=dict(class_name=object_name,run_id=task_id,run_root=str(root),boot_id=meta['boot_id'],
                        grasp_lift_verified=True,holding_opening_rad=holding,model=model,
                        body_palm_m=np.mean([r['body_palm_m'] for r in lifted],axis=0).tolist(),
                        cap_palm_m=np.mean([r['cap_palm_m'] for r in lifted],axis=0).tolist(),
                        body_minus_palm_horizontal_m=(np.asarray(m['center_m'])-sample['snapshot']['palm_base_at_depth']).tolist(),
                        last_horizontal_measurement=m)
                    if object_name=='toy_penguin':
                        if hand_contact is not None:
                            held.update(hand_face_reference=hand_contact['hand_face_reference'],
                                original_object_depth_stamp_ns=hand_contact['original_object_depth_stamp_ns'],
                                body_palm_source=hand_contact['body_palm_source'],head_palm_source=hand_contact['head_palm_source'])
                        else:
                            held['body_palm_source']='current_body_depth' if all(r['measurement'].get('current_body_depth_measured',True) for r in lifted) else 'source_rigid_body_shape_registered_to_current_head_depth'
                            held['head_palm_source']='three_current_head_depth_fits'
                    for key in ('body_palm_m','cap_palm_m'):
                        if np.max(np.linalg.norm(np.asarray([r[key] for r in lifted])-held[key],axis=1))>.02:
                            raise ValueError('pepper_not_stable_relative_to_palm')
                    report.update(grasp_success_verified=True,lift_verification=lifted,held_opening_rad=holding)
                    write_json(root/'held-context.json',held);report['confirmed_stages'].append('grasp_lift');save()
                    if grasp_only:
                        report.update(status=object_name+'_grasp_and_lift_verified',object_remains_held=True);return report
                    folder('carry');record(dict(event='current_three_frame_lift_proof_reused_before_retraction',additional_redundant_full_body_check=False))
                    stage('retract_preserving_grip');runtime.retract_holding(holding)
                    if not tucked_holding(runtime.state()['joints'],holding):raise ValueError('pepper_retraction_not_observed')
                    hold_frames('after-retract',allow_rgb=True)
                    stage('navigate_destination');backend.stage='navigate_destination'
                    current=runtime.state()
                    if backend.room_at(current['poses']['map'])!=source_room:raise ValueError('pepper_not_in_source_room_before_carry')
                    backend.navigate(source_room,destination_room,cfg['destination_staging_goal'],holding,'destination-navigation')
                    hold_frames('destination-held',allow_rgb=True)
                    report['confirmed_stages'].append('carry_destination');save()
                if continue_released_placement is not None or verify_released_placement is not None:
                    folder('place')
                    write_json(runtime.root/'placement-plan.json',plan)
                else:
                    folder('place')
                    if reuse_horizontal_placement:
                        stage('verify_current_horizontal_placement_hold')
                        write_json(runtime.root/'placement-plan.json',plan)
                        record(dict(event='current_held_placement_posture_reused_without_replaying_preparation'))
                    else:
                        stage('measure_destination_and_plan')
                        if backend.room_at(runtime.state()['poses']['map'])!=destination_room:raise ValueError('pepper_destination_room_not_confirmed')
                        for attempt in range(5):
                            sample=runtime.sample(processing_reserve_ns=200_000_000);s=sample['snapshot']
                            plane=measure_table(sample['depth'],s,destination)
                            try:runtime.fresh_measurement(s);break
                            except ValueError as exc:
                                if str(exc) not in ('fresh_carry_images_required','fresh_current_carry_joint_feedback_required') or attempt==4:raise
                                record(dict(event='expired_initial_table_frame_reacquire_without_motion',attempt=attempt,reason=str(exc)))
                        runtime.save(sample,'table-before-extension')
                        plan=make_plan(sample['depth'],s,destination,plane,held,cfg,task_id)
                        from pregrasp_geometry import expected_horizontal_center
                        predicted=map_point(s,expected_horizontal_center(plan['palm_height_m'],held['body_minus_palm_horizontal_m']))
                        clearance=extension_clearance(predicted,s,plane,plan)
                        write_json(runtime.root/'extension-clearance.json',clearance)
                        if not clearance['ready']:raise ValueError('pepper_horizontal_extension_not_outside_table')
                        write_json(runtime.root/'placement-plan.json',plan)
                        stage('prepare_horizontal_for_placement');runtime.prepare_place(plan['palm_height_m'],holding)
                    def place_observation(label):
                        nonlocal serial
                        last='no_current_pepper_and_table'
                        for _ in range(5):
                            sample=runtime.sample(processing_reserve_ns=200_000_000);s=sample['snapshot'];serial+=1
                            try:
                                if object_name=='toy_penguin' and held.get('hand_face_reference'):
                                    proof=hand_hold_evidence(sample,runtime.palm_pose(s),held,
                                        now_ns=runtime.node.get_clock().now().nanoseconds,require_pose=True)
                                else:proof=current_depth(sample['rgb'],sample['depth'],s,runtime.palm_pose(s),held)
                                plane=measure_table(sample['depth'],s,destination);state=runtime.state()
                                timing=current_images_and_feedback(s,state,runtime.node.get_clock().now().nanoseconds,runtime.epoch,
                                    joint_max_age_ns=600_000_000)
                                m=proof['measurement'];capture=runtime.save(sample,f'{serial:03d}-{label}',m)
                                return dict(measurement=m,snapshot=s,plane=plane,state=state,capture=str(capture),timing=timing,
                                    object_pose_source=m.get('position_source'),object_depth_independently_measured=
                                        bool(m.get('current_body_depth_measured',True) and m.get('current_head_depth_measured',True)))
                            except ValueError as exc:
                                last=str(exc);runtime.save(sample,f'{serial:03d}-{label}-rejected')
                        raise ValueError('pepper_current_placement_surface_missing:'+last)
                    stage('approach_measured_release_region');steps=[]
                    from task_outcome_policy import RELEASE_BOTTOM_GAP_M
                    for index in range(6):
                        row=place_observation('approach');check=region(row['measurement'],row['snapshot'],plan,row['plane'])
                        if check['ready']:break
                        if index==5:raise ValueError('pepper_five_measured_placement_corrections_exhausted')
                        if not RELEASE_BOTTOM_GAP_M[0]<=check['estimated_bottom_gap_m']<=RELEASE_BOTTOM_GAP_M[1]:
                            goal=row['state']['joints'].copy()
                            goal[0]+=cfg['policy']['release_gap_m']-check['estimated_bottom_gap_m']
                            runtime.move_joints(goal,'correct_actual_release_height')
                            plan['palm_height_m']=goal[0]+OFFSET
                            steps.append(dict(kind='height',actual_before=row['capture'],goal_lift_m=goal[0]));continue
                        step=approach_step(row['measurement'],row['snapshot'],plan,row['plane'])
                        backend.stage='approach_placement'
                        backend.adjust(f'pepper-placement-approach-{index:02d}',row['state']['poses']['map'],step['delta_m'],
                            row['state']['joints'][:5],holding,single_placement_approach=True)
                        steps.append(dict(kind='base',actual_before=row['capture'],delta_m=step['delta_m']))
                    report['placement_approaches']=steps;save()
                    stage('release_once_from_current_support');release_row=None
                    def release_check_once():
                        nonlocal release_row
                        row=place_observation('pre-release');check=region(row['measurement'],row['snapshot'],plan,row['plane'])
                        if not check['ready']:raise ValueError('pepper_release_not_over_current_safe_support')
                        body_map=map_point(row['snapshot'],np.asarray(row['measurement']['center_m']))
                        cap_map=map_point(row['snapshot'],np.asarray(row['measurement']['cap_center_m']))
                        plan['actual_release_xy_map_m']=body_map[:2].tolist()
                        plan['actual_cap_minus_body_map_m']=(cap_map-body_map).tolist()
                        if object_name=='toy_penguin':
                            plan['release_xy_source']=row['object_pose_source']
                            plan['release_object_depth_independently_measured']=row['object_depth_independently_measured']
                        release_row=dict(row,region=check)
                        write_json(runtime.root/'actual-release-support.json',dict(capture=row['capture'],region=check,
                            object_pose_source=row['object_pose_source'],object_depth_independently_measured=row['object_depth_independently_measured']))
                        current_images_and_feedback(row['snapshot'],runtime.state(),runtime.node.get_clock().now().nanoseconds,runtime.epoch,
                            joint_max_age_ns=600_000_000)
                    def release_check():
                        for attempt in range(5):
                            try:return release_check_once()
                            except ValueError as exc:
                                if str(exc) not in ('fresh_carry_images_required','fresh_current_carry_joint_feedback_required',
                                        'fresh_can_rgbd_and_joints_required') or attempt==4:raise
                                record(dict(event='expired_pepper_release_frame_reacquire_without_action',attempt=attempt,reason=str(exc)))
                    runtime.release(release_check);report['hand_released']=True
                    write_json(runtime.root/'placement-plan.json',plan);save()
                if verify_released_placement is not None:
                    cleanup=True
                    record(dict(event='placement_verification_only_no_robot_commands'))
                else:
                    stage('withdraw_and_retract_open_hand');cleanup=False
                    try:
                        runtime.raise_open();state=runtime.state();backend.stage='withdraw'
                        backend.adjust('pepper-placement-retreat',state['poses']['map'],[-cfg['policy']['retreat_m'],0.,0.],
                            state['joints'][:5],state['joints'][5])
                        runtime.tuck_open();cleanup=True
                    except Exception as exc:
                        report['cleanup_error']=str(exc);record(dict(event='verify_placed_pepper_after_cleanup_issue',reason=str(exc)))
                stage('verify_actual_placement');placed=[]
                for _ in range(9):
                    sample=runtime.sample(processing_reserve_ns=200_000_000);s=sample['snapshot'];serial+=1
                    try:
                        body,cap=expected_placed(s,plan)
                        m=measure_near(sample['rgb'],sample['depth'],s,body,cap,model)
                        if object_name=='toy_penguin' and not (m.get('current_body_depth_measured') and m.get('current_head_depth_measured')):
                            raise ValueError('new_independent_penguin_body_and_head_depth_required_after_release')
                        plane=measure_table(sample['depth'],s,destination)
                        runtime.fresh_measurement(s)
                    except ValueError as exc:
                        capture=runtime.save(sample,f'{serial:03d}-placed-rejected')
                        record(dict(event='new_placed_frame_rejected_without_motion',reason=str(exc),capture=str(capture)))
                        continue
                    if placed and m['stamp_ns']-placed[-1]['measurement']['stamp_ns']<200_000_000:continue
                    capture=runtime.save(sample,f'{serial:03d}-placed',m)
                    placed.append(dict(measurement=m,snapshot=s,plane=plane,capture=str(capture)))
                    if len(placed)==3:break
                state=runtime.state();verification=verify_placed(placed,plan,state['joints'],cleanup_completed=cleanup)
                write_json(runtime.root/'placement-verification.json',verification)
                ready=cleanup and robot_ready(dict(zip(('arm_lift_joint','arm_flex_joint','arm_roll_joint',
                    'wrist_flex_joint','wrist_roll_joint','hand_motor_joint'),state['joints'])))
                report['confirmed_stages'].append('place_verify')
                report.update(status='automatic_'+object_name+'_transfer_verified' if ready else 'automatic_'+object_name+'_transfer_verified_cleanup_required',
                    physical_transfer_verified=True,placement_success_verified=True,
                    uninterrupted_transfer_verified=(continue_open_pregrasp is None and continue_destination_hold is None
                        and continue_released_placement is None and verify_released_placement is None and continue_navigation_hold is None and continue_lifted_hold is None
                        and tuple(report['confirmed_stages'])==STAGES),
                    robot_ready_for_next_task=bool(ready),cleanup_completed=cleanup,final_placement_rows=placed,
                    object_remains_held=False)
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
                record(dict(event='stopped',stage=report.get('stage'),reason=report['reason']))
            finally:
                if worker:worker.close()
                if backend:backend.close()
                if runtime:
                    try:report['last_fresh_robot_state']=runtime.state()
                    except Exception as exc:report['state_read_error']=str(exc)
                    runtime.shutdown()
                if rclpy.ok():rclpy.shutdown()
                report['completed_ns']=time.time_ns();report['elapsed_s']=(report['completed_ns']-started)/1e9
                save();print(json.dumps(report,allow_nan=False),flush=True)
    return report


def main(argv=None,*,object_name='ground_pepper'):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');p.add_argument('--observe-only',action='store_true')
    p.add_argument('--grasp-only',action='store_true');p.add_argument('--check-inputs',action='store_true')
    p.add_argument('--task-config',type=Path)
    p.add_argument('--navigation-profile',choices=('baseline','balanced_v1','carry_fast_v1'))
    p.add_argument('--output',type=Path);p.add_argument('--task-id');p.add_argument('--confirm-layout');p.add_argument('--confirm-localization',action='store_true')
    p.add_argument('--continue-open-pregrasp',type=Path)
    p.add_argument('--continue-destination-hold',type=Path)
    p.add_argument('--continue-navigation-hold',type=Path)
    p.add_argument('--continue-lifted-hold',type=Path)
    p.add_argument('--continue-released-placement',type=Path)
    p.add_argument('--verify-released-placement',type=Path)
    p.add_argument('--simulation-ignore-collision',action='store_true');args=p.parse_args(argv)
    if args.run and args.observe_only:p.error('choose one run mode')
    if sum(x is not None for x in (args.continue_open_pregrasp,args.continue_destination_hold,args.continue_released_placement,args.verify_released_placement,args.continue_navigation_hold,args.continue_lifted_hold))>1:
        p.error('choose one continuation mode')
    if args.continue_lifted_hold is not None and (not args.run or args.grasp_only
            or not args.continue_lifted_hold.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))):
        p.error('lifted hold continuation requires --run and reviewed held D-drive predecessor')
    if args.continue_navigation_hold is not None and (not args.run or args.grasp_only
            or not args.continue_navigation_hold.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))):
        p.error('navigation continuation requires --run and reviewed held D-drive predecessor')
    if args.verify_released_placement is not None and (not args.run or args.grasp_only
            or not args.verify_released_placement.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))):
        p.error('placement verification requires --run and a reviewed D-drive predecessor')
    if args.continue_released_placement is not None and (not args.run or args.grasp_only
            or not args.continue_released_placement.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))):
        p.error('released placement continuation requires --run and a reviewed D-drive predecessor')
    if args.continue_destination_hold is not None and (not args.run or args.grasp_only
            or not args.continue_destination_hold.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))):
        p.error('destination continuation requires --run and a reviewed unreleased D-drive predecessor')
    if args.continue_open_pregrasp is not None and (not args.run
            or not args.continue_open_pregrasp.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))):
        p.error('open pregrasp continuation requires --run and a reviewed D-drive predecessor')
    if object_name=='toy_penguin' and any(x is not None for x in (args.continue_open_pregrasp,args.continue_destination_hold,args.continue_navigation_hold,args.continue_lifted_hold,args.continue_released_placement,args.verify_released_placement)):
        p.error('toy_penguin currently supports new observation/full-run tasks only; inspect actual state after interruption')
    if args.check_inputs:
        from apple_transfer_automation import check_inputs
        if object_name=='toy_penguin':from penguin_tasks import load_task,configured_transfer,task_digest
        else:from pepper_tasks import load_task,configured_transfer,task_digest
        import yaml
        baseline=json.loads((Path(__file__).parent/'config/apple_transfer_auto_v1.json').read_text())
        task=load_task(args.task_config)
        configured_transfer(baseline,task,yaml.safe_load(Path(baseline['environment']).read_text()),
            json.loads(Path(baseline['semantic_map']).read_text())['destinations'])
        print(json.dumps(dict(check_inputs(baseline),configured_task=task,task_sha256=task_digest(task))));return
    if not args.run and not args.observe_only:
        if object_name=='toy_penguin':from penguin_tasks import load_task
        else:from pepper_tasks import load_task
        print(json.dumps(dict(status=object_name+'_plan_only',class_name=object_name,stages=STAGES,
            robot_commands_sent=False,physical_grasp_verified=False,source_geometry='observed_body_and_head_with_head_contact' if object_name=='toy_penguin' else 'observed_wide_body_and_cap',
            navigation_parameters_changed=False,configured_task=load_task(args.task_config))));return
    if (args.output is None or not args.output.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))
            or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or args.run and (args.confirm_layout!='LayoutA' or not args.confirm_localization or not args.simulation_ignore_collision)):
        p.error('same reviewed LayoutA/domain71/localhost simulation arguments and NEW D-drive output required')
    task_id=args.task_id or ('penguin-' if object_name=='toy_penguin' else 'pepper-')+uuid.uuid4().hex
    if not task_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in task_id):p.error('invalid task id')
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    result=run_live(args.output,task_id,observe_only=args.observe_only,grasp_only=args.grasp_only,
        continue_open_pregrasp=args.continue_open_pregrasp,continue_destination_hold=args.continue_destination_hold,
        continue_released_placement=args.continue_released_placement,verify_released_placement=args.verify_released_placement,
        task_config=args.task_config,continue_navigation_hold=args.continue_navigation_hold,continue_lifted_hold=args.continue_lifted_hold,object_name=object_name,navigation_profile=args.navigation_profile)
    if result['status'].startswith('stopped'):raise SystemExit(1)

if __name__=='__main__':main()

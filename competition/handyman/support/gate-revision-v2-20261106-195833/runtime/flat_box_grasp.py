"""Flat-cigarette-box grasp with current RGBD and the measured finger triggers.

Reuses finite transport, ownership, speed and closure handling. Cylinder shape
assumptions are replaced with a measured top plane and box dimensions.
"""
from competition_paths import is_run_path
import argparse,json,math,os,signal,time,uuid
from pathlib import Path
import cv2
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from horizontal_can_trial import HorizontalCanRuntime
from flat_box_geometry import estimate,appearance_from_source,measure_near,operator_confirmed_source,select_source
from flat_box_partial_top import observe_known
from outcome_transfer_policy import load as outcome_policy,snapshot as policy_snapshot,alignment_tolerances
from outcome_transfer_runtime import OutcomeRuntime
from apple_transfer_automation import RobotLock,write_json
from apple_worker_client import Worker
from hand_camera_ray import rotation
from canned_grasp_geometry import transform_point
from inverse_table_place_plan import OFFSET,FLEX,WRIST
from task_outcome_policy import pose_matches,hand_is_open
from pepper_visual_motion import _features,fit_rigid
from table_approach_reachability import base_path
from thin_box_clearance import ThinBoxClearance,plane_in_map
from horizontal_can_grasp_plan import forward
from apple_transfer_auto_plan import map_point,footprint
from head_joint_feedback import TaskHeadFeedback
from can_destination_surface import measure as measure_table

FINGER_TRIGGER_LOCAL=np.array([.0000000685,.00005,.0621258659])


class BoxRuntime(OutcomeRuntime,TaskHeadFeedback,HorizontalCanRuntime):
    default_observation_profile="rgbd"
    def controller_delta(self,delta):
        value=super().controller_delta(delta)
        if getattr(self,'source_table',None) is not None:
            reach=base_path(self.state()['poses']['map'],value,getattr(self,'source_reachability_table',self.source_table))
            self.record(dict(event='box_approach_base_reachability',reachability=reach))
            if not reach['ready']:raise ValueError('source_target_not_reachable_without_base_entering_table')
        return value
    def accept_box(self,m,s):
        if m.get('class_name')!='cigarette' or m['stamp_ns']!=s['streams']['depth']['stamp_ns']:
            raise ValueError('same_frame_box_depth_required')
        self.current_box=m

    def contact_error(self,center,snapshot):
        m=self.current_box
        if m['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']:raise ValueError('old_box_measurement')
        palm=self.palm_pose(snapshot)
        return np.asarray(m['grasp_point_base_m'])-(np.asarray(palm['translation'])+rotation(palm['quaternion'])@FINGER_TRIGGER_LOCAL)

    def is_aligned(self,center,snapshot):
        self.contact_error(center,snapshot)  # enforce same-frame measurement
        if hasattr(self,'box_safety'):
            axis=rotation(snapshot['tracking_transforms']['odom_from_base_at_depth']['quaternion']).T@self.box_axis_odom
            proof=self.box_safety.closure_region(self.palm_pose(snapshot),center,axis,self.box_model,**outcome_policy()['closure_region'])
            self.record(dict(event='actual_finger_closure_region',proof=proof))
            return bool(proof['ready'])
        error=abs(self.contact_error(center,snapshot));tolerance=alignment_tolerances(self.open_side_margin_m)
        return bool((error<=tolerance).all())

    def approach(self,observe,expected_arm,index):
        def checked():
            sample,m=observe()
            if m.get('position_xy_is_prediction') and not self.is_aligned(m['center_m'],sample['snapshot']):
                raise ValueError('locked_contact_pose_not_aligned_no_blind_approach')
            return sample,m
        return super().approach(checked,expected_arm,index)

    def approach_plan(self,error):
        x,y,z=map(float,error)
        lateral_limit=max(0.,self.open_side_margin_m-outcome_policy()['closure_region']['side_clearance'])
        if abs(y)>lateral_limit:
            safe_shift=max(.002,self.open_side_margin_m-.001)
            if abs(y)<=safe_shift and abs(x)<.07:return dict(phase='small_open_finger_centering',delta=[0.,y,0.])
            clearance=.09
            if x<clearance-.01:return dict(phase='clear_before_lateral_alignment',delta=[x-clearance,0.,0.])
            return dict(phase='lateral_alignment',delta=[0.,y,0.])
        return dict(phase='straight_contact_approach' if x>0 else 'measured_contact_retreat',delta=[x,0.,0.])


class BoxMotionAnchor:
    """Static background predicts ROI/direction; current box depth is re-fit."""
    def __init__(self,sample,measurement):self.sample=sample;self.measurement=measurement;self.features=None
    def predict(self,current,expected):
        old=self.sample['snapshot'];new=current['snapshot']
        if old['tracking_epoch']!=new['tracking_epoch'] or new['streams']['depth']['stamp_ns']<=old['streams']['depth']['stamp_ns']:
            raise ValueError('same_task_new_box_frame_required')
        ot=old['tracking_transforms']['odom_from_base_at_depth'];nt=new['tracking_transforms']['odom_from_base_at_depth']
        step=rotation(ot['quaternion']).T@(np.asarray(nt['translation'])-ot['translation'])
        if np.linalg.norm(step[:2])<.003:return expected,None
        if self.features is None:self.features=_features(self.sample,np.asarray(self.measurement['center_m']))
        a,da=self.features
        b,db=_features(current,np.asarray(expected))
        pairs=[pair[0] for pair in cv2.BFMatcher().knnMatch(da,db,k=2) if len(pair)==2 and pair[0].distance<.65*pair[1].distance]
        if len(pairs)<15:raise ValueError('box_static_background_not_matched')
        r,t,e=fit_rigid(a[[p.queryIdx for p in pairs]],b[[p.trainIdx for p in pairs]])
        predicted=r@np.asarray(self.measurement['center_m'])+t
        distance=float(np.linalg.norm(step[:2]))
        if np.linalg.norm(predicted-expected)>max(.08,.25*distance):raise ValueError('box_roi_motion_disagreement')
        actual=-r.T@t
        e.update(reference_epoch=old['tracking_epoch'],reference_depth_stamp_ns=old['streams']['depth']['stamp_ns'],
                 current_depth_stamp_ns=new['streams']['depth']['stamp_ns'],prediction_only=True)
        if distance>=.02 and .7<=np.linalg.norm(actual[:2])/distance<=1.3:
            angle=math.atan2(actual[1],actual[0])-math.atan2(step[1],step[0]);angle=math.atan2(math.sin(angle),math.cos(angle))
            if abs(angle)<=.35:e['controller_to_visual_angle_rad']=angle
        return predicted,e


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',action='store_true');parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source-table-id')
    parser.add_argument('--operator-confirmed-reference',type=Path)
    parser.add_argument('--gripper-geometry',type=Path)
    parser.add_argument('--continue-open',type=Path)
    parser.add_argument('--tracked-source',type=Path)
    args=parser.parse_args()
    if not args.run:print(json.dumps(dict(status='flat_box_plan_only',robot_commands_sent=False)));return
    if (os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or not is_run_path(args.output.resolve())):raise ValueError('domain71_D_output_required')
    root=args.output;runtime=worker=None;serial=0
    report=dict(status='starting',class_name='cigarette',close_commands_sent=0,release_commands_sent=0,
        approach_commands_sent=0,grasp_success_verified=False,model_changed=False,navigation_parameters_changed=False,
        operator_confirmed_test_object=bool(args.operator_confirmed_reference),identity_threshold_unchanged=True)
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    with RobotLock():
        root.mkdir(exist_ok=False);start=time.time_ns();policy=policy_snapshot(root)
        with (root/'journal.jsonl').open('x') as stream:
            def record(event):
                if event.get('event')=='can_closure_sent':report['close_commands_sent']+=1
                if event.get('event')=='finite_can_approach_sent':report['approach_commands_sent']+=1
                stream.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');stream.flush();write_json(root/'result.json',report)
            def stage(name):report['stage']=name;write_json(root/'result.json',report)
            try:
                import rclpy
                rclpy.init();epoch='flat-box-'+uuid.uuid4().hex
                runtime=BoxRuntime(root,epoch,record);sample=runtime.sample(processing_reserve_ns=0)
                state=runtime.state();runtime.ownership();report['initial_state_before_localization']=state
                from table_localization_client import localize
                stage('verify_source_table_localization');report['localization']=localize(runtime,root,'source')
                sample=runtime.sample(processing_reserve_ns=0);state=runtime.state();report['initial_state']=state
                if args.continue_open:
                    if not is_run_path(args.continue_open.resolve()):raise ValueError('D_predecessor_required')
                    previous=json.loads((args.continue_open/'result.json').read_text())
                    old=previous.get('last_robot_state',{})
                    if (previous.get('close_commands_sent')!=0 or previous.get('release_commands_sent')!=0
                            or previous.get('source_table_verified')!=args.source_table_id
                            or not hand_is_open(state['joints'][5])):
                        raise ValueError('same_ungrasped_open_predecessor_required')
                    report['continued_from_open_predecessor']=str(args.continue_open)
                elif pose_matches(state['joints'],[0,0,0,-math.pi/2,0,0.]):
                    empty=state['joints'].copy();empty[5]=.65;runtime.move_joints(empty,'open_fully_closed_empty_hand',hand=True)
                elif not hand_is_open(state['joints'][5]):raise ValueError('new_grasp_requires_empty_open_hand')
                cfg=json.loads((runtime.code/'config/apple_transfer_auto_v1.json').read_text())
                if args.source_table_id:
                    runtime.source_table=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==args.source_table_id)
                    yaw=state['poses']['map'][2];target=min((0.,math.pi/2,math.pi,-math.pi/2),key=lambda a:abs(math.atan2(math.sin(a-yaw),math.cos(a-yaw))))
                    angle=math.atan2(math.sin(target-yaw),math.cos(target-yaw))
                    # Actual RGBD/TF alignment replaces a redundant cardinal-yaw move.
                    runtime.face_source((0.,-.5))
                worker=Worker(cfg['model_python'],cfg['registry'],root/'model.json',imgsz=640)
                ref=None
                if args.operator_confirmed_reference:
                    if not is_run_path(args.operator_confirmed_reference.resolve()):raise ValueError('D_reference_required')
                    ref=json.loads((args.operator_confirmed_reference/'held-context.json').read_text())
                def acquire_source(attempt):
                    sample=runtime.sample(processing_reserve_ns=0,force_new=True)
                    capture=runtime.save(sample,'source' if attempt==0 else f'source-retry-{attempt}')
                    detections=worker.infer(capture,pump=runtime.pump)
                    write_json(capture/'detections.json',detections)
                    return sample,detections
                def solve_source(sample,detections):
                    if args.source_table_id:
                        source_plane=measure_table(sample['depth'],sample['snapshot'],runtime.source_table)
                        runtime.source_reachability_table=runtime.table_geometry(sample,runtime.source_table,source_plane)
                    targets=[d for d in detections['detections'] if d['name']=='cigarette']
                    if targets:
                        model,selection=select_source(sample['rgb'],sample['depth'],sample['snapshot'],targets,getattr(runtime,'source_table',None),check_nominal_reach=False)
                        report.update(source_model_detection_used=True,source_selection=selection)
                    elif args.tracked_source:
                        if not is_run_path(args.tracked_source.resolve()):raise ValueError('D_task_reference_required')
                        from tracked_box_source import reacquire
                        tracked=json.loads(args.tracked_source.read_text())
                        model=reacquire(sample['rgb'],sample['depth'],sample['snapshot'],tracked,runtime.source_table)
                        report.update(source_model_detection_used=False,source_identity=model['identity_source'],model_miss_recorded=True,tracked_source=str(args.tracked_source))
                    elif args.operator_confirmed_reference and args.source_table_id:
                        model=operator_confirmed_source(sample['rgb'],sample['depth'],sample['snapshot'],runtime.source_table,ref['appearance'],ref['model'])
                        report.update(source_model_detection_used=False,source_identity='explicit_operator_confirmed_retest_colour_and_actual_depth',model_miss_recorded=True)
                    else:raise ValueError('one_current_cigarette_detection_required')
                    return model
                from source_geometry_recovery import acquire_measured_source
                sample,detections,model=acquire_measured_source(acquire_source,solve_source,runtime.face_source,record)
                if args.source_table_id:
                    center_map=map_point(sample['snapshot'],np.asarray(model['center_m']));table_center,table_half=footprint(runtime.source_table)
                    if not (abs(center_map[:2]-table_center)<=table_half).all():raise ValueError('cigarette_not_on_requested_source_table')
                    report['source_table_verified']=args.source_table_id
                axis=np.asarray(model['axis_base']);turn=math.atan2(axis[1],axis[0])-math.pi/2
                turn=math.atan2(math.sin(turn),math.cos(turn))
                other=math.atan2(math.sin(turn+math.pi/2),math.cos(turn+math.pi/2))
                other=other-math.copysign(math.pi,other) if abs(other)>math.pi/2 else other
                turn=turn-math.copysign(math.pi,turn) if abs(turn)>math.pi/2 else turn
                if abs(other)<abs(turn):turn=other
                if abs(turn)>.02 and args.operator_confirmed_reference and args.source_table_id:
                    if abs(turn)>.35:raise ValueError('box_heading_requires_new_staging')
                    runtime.base_move([0.,0.,turn],runtime.state()['joints'][:5],'align_open_jaws_with_observed_box_long_edge')
                    sample=runtime.sample(processing_reserve_ns=0);runtime.save(sample,'source-heading-aligned')
                    model=operator_confirmed_source(sample['rgb'],sample['depth'],sample['snapshot'],runtime.source_table,ref['appearance'],ref['model'])
                write_json(root/'source-snapshot.json',sample['snapshot'])
                appearance=(dict(ref['appearance'],source='verified_same_operator_confirmed_box_with_current_colour_depth_match')
                    if ref is not None and model.get('operator_confirmed_class') else appearance_from_source(sample['rgb'],sample['depth'],sample['snapshot'],model))
                if model['height_m']<=.035:
                    if not args.gripper_geometry:raise ValueError('thin_object_requires_physical_clearance_plan')
                    safety=ThinBoxClearance(args.gripper_geometry)
                    table_plane_map=np.asarray(measure_table(sample['depth'],sample['snapshot'],runtime.source_table)['plane_map'] if getattr(runtime,'source_table',None) else plane_in_map(sample['snapshot'],model['support_plane_base']),float)
                    _,flat_r=forward([.25,FLEX,0.,WRIST,0.,.65])
                    target_palm=np.asarray(model['center_m'])-flat_r@FINGER_TRIGGER_LOCAL
                    safe=safety.proposal(sample['snapshot'],model['center_m'],model['height_m'],target_palm,flat_r,table_plane_map)
                    if not safe['feasible']:raise ValueError('thin_box_not_graspable_without_table_contact')
                    target_height=(safe['minimum_allowed_palm_height_m']+safe['maximum_palm_height_for_contact_m'])/2
                    center_map=map_point(sample['snapshot'],np.asarray(model['center_m']))
                    support_map=float(np.r_[center_map[:2],1.]@table_plane_map)
                    model['grip_above_bottom_m']=float(target_height-support_map)
                    model['grasp_point_base_m'][2]=model['support_height_m']+model['grip_above_bottom_m']
                    write_json(root/'thin-object-clearance-plan.json',dict(safe,target_palm_height_map_m=target_height,
                        grip_above_bottom_m=model['grip_above_bottom_m'],table_plane_map=table_plane_map.tolist()))
                    report['thin_object_no_table_contact_policy']=True
                closing_width=abs(model['axis_base'][1])*model['length_m']+abs(model['axis_base'][0])*model['width_m']
                runtime.open_side_margin_m=(.093026-closing_width)/2
                if runtime.open_side_margin_m<.002:raise ValueError('box_too_wide_for_current_front_grasp')
                runtime.lateral_tolerance_m=alignment_tolerances(runtime.open_side_margin_m)[1]
                write_json(root/'source-geometry.json',model);write_json(root/'appearance.json',appearance)
                stage('prepare_flat_arm_for_thin_box')
                goal=runtime.state()['joints'];goal[0]=.4428021848 if model['height_m']<=.035 else model['grasp_point_base_m'][2]-OFFSET
                if not args.continue_open:
                    runtime.move_joints(goal,'raise_before_thin_box_extension' if model['height_m']<=.035 else 'set_finger_side_overlap_height')
                    goal[1]=FLEX;goal[3]=WRIST;runtime.move_joints(goal,'extend_flat_open_arm')
                if model['height_m']<=.035 and not args.continue_open:
                    goal[0]=model['grasp_point_base_m'][2]-OFFSET;runtime.move_joints(goal,'lower_flat_hand_to_physical_clearance_height')
                runtime.face_source((0.,-.65))
                current_center=transform_point(sample['snapshot'],model['center_m']);anchor=None
                source_axis_odom=rotation(sample['snapshot']['tracking_transforms']['odom_from_base_at_depth']['quaternion'])@np.asarray(model['axis_base'])
                if model['height_m']<=.035:runtime.box_safety=safety;runtime.box_model=model;runtime.box_axis_odom=source_axis_odom
                def observe():
                    nonlocal current_center,anchor,serial
                    last=None
                    for _ in range(3):
                        sample_=runtime.sample(processing_reserve_ns=100_000_000);s=sample_['snapshot']
                        expected=transform_point(s,current_center,inverse=True);motion=None
                        try:
                            if anchor is not None:
                                try:expected,motion=anchor.predict(sample_,expected)
                                except ValueError as exc:record(dict(event='box_visual_motion_unavailable',reason=str(exc)))
                            axis=rotation(s['tracking_transforms']['odom_from_base_at_depth']['quaternion']).T@source_axis_odom
                            palm=runtime.palm_pose(s);contact=np.asarray(palm['translation'])+rotation(palm['quaternion'])@FINGER_TRIGGER_LOCAL
                            m=observe_known(sample_['rgb'],sample_['depth'],s,expected,axis,model,appearance,allow_locked=np.linalg.norm(expected[:2]-contact[:2])<=.035)
                            if model['height_m']<=.035:
                                if getattr(runtime,'source_table',None):
                                    observed_plane=measure_table(sample_['depth'],s,runtime.source_table)
                                    runtime.source_reachability_table=runtime.table_geometry(sample_,runtime.source_table,observed_plane)
                                    current_plane=observed_plane['plane_map']
                                else:current_plane=table_plane_map
                                proof=safety.check(s,runtime.palm_pose(s),m['center_m'],model['height_m'],current_plane)
                                if proof['actual_palm_height_map_m']>proof['maximum_palm_height_for_contact_m']:raise ValueError('actual_fingers_above_thin_box_contact_band')
                                record(dict(event='thin_box_table_clearance_checked',proof=proof))
                            runtime.accept_box(m,s)
                            if motion is not None:runtime.accept_visual_motion(motion,s);record(dict(event='box_static_motion',**motion))
                            runtime.fresh_measurement(s);current_center=transform_point(s,m['center_m'])
                            anchor=BoxMotionAnchor(sample_,m);serial+=1
                            capture_=runtime.save(sample_,f'box-{serial:03d}',m)
                            record(dict(event='actual_box_depth',capture=str(capture_),measurement=m))
                            return sample_,m
                        except ValueError as exc:last=str(exc)
                    raise ValueError(last or 'current_flat_box_missing')
                stage('align_then_full_forward_approach')
                expected_arm=runtime.state()['joints'][:5]
                for index in range(policy['maximum_corrections']+1):
                    result=runtime.approach(observe,expected_arm,index)
                    if result.get('status')=='already_aligned':break
                else:
                    sample_,m=observe()
                    if not runtime.is_aligned(m['center_m'],sample_['snapshot']):raise ValueError('initial_plus_five_corrections_exhausted')
                stage('close_once')
                closure,before,m=runtime.close_gripper(observe,expected_arm)
                holding=float(closure['closure_evidence']['observed_opening_rad']);report['holding_opening_rad']=holding
                baseline,before_m=observe();c0=transform_point(baseline['snapshot'],before_m['center_m'])
                p0=transform_point(baseline['snapshot'],baseline['snapshot']['palm_base_at_depth'])
                v0=transform_point(baseline['snapshot'],before_m.get('visible_surface_centroid_m',before_m['center_m']))
                stage('lift_and_verify_actual_box_depth')
                goal=runtime.state()['joints'];goal[0]+=.08;goal[5]=holding
                runtime.move_joints(goal,'lift_flat_box')
                rows=[]
                for index in range(policy['verification']['lift_frames']):
                    if index:runtime.pump(policy['verification']['frame_separation_s'])
                    sample_=runtime.sample(processing_reserve_ns=0,profile='paired',force_new=True);s=sample_['snapshot']
                    p1=transform_point(s,s['palm_base_at_depth']);predicted=transform_point(s,c0+p1-p0,inverse=True)
                    axis=rotation(s['tracking_transforms']['odom_from_base_at_depth']['quaternion']).T@source_axis_odom
                    measured=observe_known(sample_['rgb'],sample_['depth'],s,predicted,axis,model,appearance,allow_locked=True)
                    c1=transform_point(s,measured.get('visible_surface_centroid_m',measured['center_m']));delta=c1-v0;error=float(np.linalg.norm(delta-(p1-p0)))
                    path=runtime.save(sample_,f'lift-proof-{index}',measured)
                    if delta[2]<policy['verification']['minimum_lift_m'] or error>policy['verification']['follow_error_m']:raise ValueError('actual_flat_box_did_not_follow_palm_lift')
                    rows.append(dict(capture=str(path),measurement=measured,observed_lift_m=float(delta[2]),follow_error_m=error))
                s=sample_['snapshot'];palm=runtime.palm_pose(s)
                held=dict(class_name='cigarette',grasp_lift_verified=True,holding_opening_rad=holding,
                    center_palm_m=(rotation(palm['quaternion']).T@(np.asarray(measured['center_m'])-palm['translation'])).tolist(),
                    model=model,appearance=appearance,source_run=str(root),source='actual_visible_RGBD_lift_observations_with_locked_XY_when_occluded')
                write_json(root/'held-context.json',held)
                report.update(status='flat_box_grasp_and_lift_verified',grasp_success_verified=True,lift_evidence=rows,
                    object_remains_held=True,final_state=runtime.state())
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
                if runtime:
                    try:report['last_robot_state']=runtime.state()
                    except Exception:pass
            finally:
                if worker:worker.close()
                if runtime:runtime.shutdown();rclpy.shutdown()
                report['elapsed_s']=(time.time_ns()-start)/1e9
                write_json(root/'result.json',report);print(json.dumps(report))

if __name__=='__main__':main()

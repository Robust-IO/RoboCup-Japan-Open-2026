"""Continue the open horizontal-can trial with the existing hand raster model.

No detector threshold edits or hidden depth. The close target is locked after
visual alignment for the vertical-only descent, as requested by the operator.
"""
from competition_paths import is_run_path
import argparse,json,math,os,signal,time,uuid
from pathlib import Path
import cv2
import numpy as np
from apple_transfer_automation import RobotLock,write_json
from horizontal_can_trial import HorizontalCanRuntime
from horizontal_can_hand import fit,mask_body,project,follows_lift,hand_feedback_fresh
from horizontal_can_grasp_plan import contact
from task_outcome_policy import pose_matches,hand_is_open
from hand_camera_ray import rotation
from canned_grasp_geometry import transform_point


class HandCanRuntime(HorizontalCanRuntime):
    def fresh_measurement(self,snapshot):
        state=self.state()
        return hand_feedback_fresh(snapshot,state,self.node.get_clock().now().nanoseconds,self.epoch)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--previous',type=Path,required=True);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--continue-open-contact',action='store_true')
    args=p.parse_args()
    if not args.run:print(json.dumps(dict(robot_commands_sent=False,status='hand_can_plan_only')));return
    base=Path('/__HANDYMAN_PACKAGE__/runtime')
    if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('isolated_domain71_required')
    for path in (args.output,args.previous,args.source):
        if not is_run_path(path.resolve()):raise ValueError('D_paths_required')
    report=dict(status='starting',close_commands_sent=0,release_commands_sent=0,approach_commands_sent=0,
        grasp_success_verified=False,physical_transfer_verified=False,operator_confirmed_identity=True,
        model_changed=False,navigation_parameters_changed=False,source_reference=str(args.source),
        previous_trial=str(args.previous),hand_pose_source='known_size_RGB_model_not_depth_sensor')
    runtime=None;serial=0;root=args.output
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    with RobotLock():
        root.mkdir(exist_ok=False);start=time.time_ns()
        with (root/'journal.jsonl').open('x') as stream:
            def record(event):
                if event.get('event')=='can_closure_sent':report['close_commands_sent']+=1
                if event.get('event')=='horizontal_base_sent':report['approach_commands_sent']+=1
                stream.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');stream.flush()
                write_json(root/'result.json',report)
            def stage(name):report['stage']=name;write_json(root/'result.json',report)
            try:
                import rclpy
                rclpy.init();runtime=HandCanRuntime(root,'horizontal-hand-'+uuid.uuid4().hex,record)
                runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
                old=json.loads((args.previous/'result.json').read_text())
                if (old.get('close_commands_sent')!=0 or old.get('release_commands_sent')!=0
                        or not pose_matches(state['joints'],old['last_robot_state']['joints'])
                        or not hand_is_open(state['joints'][5])
                        or math.dist(state['poses']['odom'][:2],old['last_robot_state']['poses']['odom'][:2])>.03):
                    raise ValueError('actual_open_ungrasped_predecessor_required')
                model=json.loads((args.source/'source-geometry.json').read_text())
                appearance=json.loads((args.source/'body-appearance.json').read_text())
                model['coloured_length_m']=model['observed_length_m']*.92
                report['initial_state']=state;report['model']=model
                target_height=float(model['center_base_m'][2]);axis_hint=np.asarray(model['axis_base'])
                def observe():
                    nonlocal serial,axis_hint
                    sample=runtime.sample(processing_reserve_ns=200_000_000)
                    m=fit(sample['hand'],sample['snapshot'],model,appearance,height=target_height,axis_hint=axis_hint)
                    axis_hint=np.asarray(m['axis_base']);runtime.current_axis=axis_hint
                    runtime.fresh_measurement(sample['snapshot']);serial+=1
                    path=runtime.save(sample,f'hand-align-{serial:03d}',m)
                    record(dict(event='same_can_hand_pose_measured',capture=str(path),measurement=m))
                    return sample,m
                if args.continue_open_contact:
                    if old.get('stage')!='close_once' or old.get('source_reference')!=str(args.source):
                        raise ValueError('matching_open_contact_predecessor_required')
                    lock=json.loads((args.previous/'locked-target.json').read_text())
                    center_odom=np.asarray(lock['center_odom_m']);axis_odom=np.asarray(lock['axis_odom'])
                    m=lock['observation'];goal=runtime.state()['joints']
                    report['continued_from_open_contact']=str(args.previous)
                    write_json(root/'locked-target.json',dict(lock,continued_without_replaying_descent=True))
                else:
                    stage('hand_visual_alignment')
                    # Earlier initial large approach is already complete. At
                    # most five more positioning actions are allowed.
                    arm=state['joints'][:5]
                    for correction in range(6):
                        sample,m=observe();error=runtime.contact_error(m['center_m'],sample['snapshot'])
                        a=runtime.current_axis;n=np.array([-a[1],a[0],0.])
                        if abs(error@n)<=.004 and abs(error@a)<=.020:break
                        if correction==5:raise ValueError('five_hand_alignment_corrections_exhausted')
                        runtime.base_move([*error[:2],0.],arm,f'hand-align-{correction}')
                    center_odom=transform_point(sample['snapshot'],m['center_m'])
                    r=rotation(sample['snapshot']['tracking_transforms']['odom_from_base_at_depth']['quaternion'])
                    axis_odom=r@runtime.current_axis
                    lock=dict(observation=m,center_odom_m=center_odom.tolist(),axis_odom=axis_odom.tolist(),
                        source_hand_stamp_ns=m['stamp_ns'],position_locked_during_vertical_only_descent=True)
                    write_json(root/'locked-target.json',lock)
                    stage('vertical_descent_to_locked_can')
                    goal=runtime.state()['joints'];goal[0]+=float(error[2])
                    runtime.move_joints(goal,'descend_to_locked_horizontal_body')
                def observe_locked():
                    sample_=runtime.sample(processing_reserve_ns=200_000_000);s=sample_['snapshot']
                    target=transform_point(s,center_odom,inverse=True)
                    axis=rotation(s['tracking_transforms']['odom_from_base_at_depth']['quaternion']).T@axis_odom
                    mask,pixels=mask_body(sample_['hand'],appearance,allow_clipped=True)
                    expected_pixel=project(s,[target])[0]
                    hull=cv2.convexHull(pixels.astype(np.float32))
                    if cv2.pointPolygonTest(hull,tuple(map(float,expected_pixel)),True)<-20:
                        raise ValueError('locked_can_not_seen_under_current_gripper')
                    runtime.current_axis=axis
                    measurement=dict(m,center_m=target.tolist(),axis_base=axis.tolist(),
                        source='locked_last_hand_visual_pose_during_vertical_only_descent',
                        position_is_prediction=True,current_object_depth_measured=False,
                        current_hand_colour_pixels=int(mask.sum()),current_hand_stamp_ns=s['streams']['hand']['stamp_ns'],
                        source_hand_stamp_ns=m['stamp_ns'])
                    return sample_,measurement
                stage('close_once')
                closure,preclose,measurement=runtime.close_gripper(observe_locked,goal[:5])
                holding=float(closure['closure_evidence']['observed_opening_rad'])
                report['holding_opening_rad']=holding
                baseline=runtime.sample(processing_reserve_ns=0);baseline_path=runtime.save(baseline,'closed-before-lift')
                stage('lift_and_verify_same_can')
                before=runtime.state();target=before['joints'].copy();target[0]+=.08;target[5]=holding
                runtime.move_joints(target,'lift_horizontal_can_from_above')
                after=runtime.state();actual_lift=float(after['joints'][0]-before['joints'][0])
                if actual_lift<.03:raise ValueError('actual_arm_lift_not_observed')
                rows=[]
                for index in range(3):
                    sample_=runtime.sample(processing_reserve_ns=0)
                    path=runtime.save(sample_,f'lift-proof-{index}')
                    proof=follows_lift(baseline['hand'],sample_['hand'],appearance)
                    row=dict(proof,actual_arm_lift_m=actual_lift,
                        before_capture=str(baseline_path),grip=runtime.state()['joints'][5])
                    if abs(row['grip']-holding)>.03:raise ValueError('grip_changed_after_lift')
                    try:
                        metric=fit(sample_['hand'],sample_['snapshot'],model,appearance,axis_hint=axis_hint)
                        row['known_size_pose']=metric
                        row['estimated_can_lift_m']=metric['center_m'][2]-target_height
                    except ValueError as exc:row['metric_pose_unavailable']=str(exc)
                    row['capture']=str(path);rows.append(row)
                report.update(status='horizontal_can_grasp_and_lift_verified',grasp_success_verified=True,
                    lift_evidence=rows,final_state=runtime.state(),base_moved_during_lift=False,
                    object_depth_measured_after_lift=False,
                    verification='same_can_texture_and_size_followed_hand_through_actual_8cm_arm_lift')
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
                if runtime:
                    try:report['last_robot_state']=runtime.state()
                    except Exception:pass
            finally:
                if runtime:runtime.shutdown();rclpy.shutdown()
                report['elapsed_s']=(time.time_ns()-start)/1e9
                write_json(root/'result.json',report);print(json.dumps(report))

if __name__=='__main__':main()

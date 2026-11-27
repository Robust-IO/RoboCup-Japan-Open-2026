"""Explicit operator-confirmed horizontal-can grasp trial; no navigation/release.

This new top-down adapter leaves all established transfer entries unchanged.
Defaults to a plan; --run is required. Every action has new measured feedback.
"""
from competition_paths import is_run_path
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid
import numpy as np

from horizontal_can_grasp_plan import validate_goal,overhead_plan,forward,contact
from horizontal_can_geometry import estimate,body_appearance,measure_near
from pepper_runtime import PepperRuntime,arm_goal_with_current_hand
from canned_grasp_runtime import NAMES,TOPICS
from apple_transfer_automation import RobotLock,write_json
from apple_worker_client import Worker
from hand_camera_ray import rotation
from motion_speed import arm_duration,finite_duration
from reviewed_carry_adjustment import endpoint
from task_outcome_policy import hand_is_open,pose_matches


class HorizontalCanRuntime(PepperRuntime):
    """Dedicated joint workspace for top-down; old pepper limits stay intact."""
    def move_joints(self,goal,label,*,hand=False,before_publish=None):
        goal=validate_goal(goal);start=self.state();topic=TOPICS[3 if hand else 2]
        if not hand:goal=arm_goal_with_current_hand(goal,start['joints'])
        if hand and not pose_matches(goal[:5]+[0.],start['joints'][:5]+[0.]):
            raise ValueError('hand_step_must_preserve_arm')
        duration=arm_duration(self.profile,'hand' if hand else 'arm',start['joints'],goal,
            holding=.02<start['joints'][5]<.62,segmented=True)
        pub=self.publisher(topic)
        try:
            if before_publish:before_publish()
            self.ownership(topic)
            self.record(dict(event='horizontal_joint_intent',label=label,goal=goal,duration_s=duration))
            pub.publish(self.trajectory([NAMES[5]] if hand else NAMES[:5],goal[5:] if hand else goal[:5],duration))
            self.record(dict(event='horizontal_joint_sent',label=label,goal=goal))
            sent=time.monotonic();stable=None
            while time.monotonic()-sent<duration+5:
                self.pump(.02);actual=self.state();self.ownership(topic)
                if not hand and abs(actual['joints'][5]-start['joints'][5])>.03:
                    raise ValueError('grip_changed_during_horizontal_arm_motion')
                if pose_matches(actual['joints'],goal,endpoint=True):
                    stable=time.monotonic() if stable is None else stable
                    if time.monotonic()-sent>=duration and time.monotonic()-stable>=.3:
                        return dict(final_state=actual,goal=goal,duration_s=duration)
                else:stable=None
            raise ValueError('horizontal_joint_endpoint_not_observed:'+label)
        finally:self.node.destroy_publisher(pub)

    def contact_error(self,center,snapshot):
        return np.asarray(center)-contact(self.palm_pose(snapshot))

    def is_aligned(self,center,snapshot):
        error=self.contact_error(center,snapshot);a=self.current_axis
        n=np.array([-a[1],a[0],0.])
        return bool(abs(error@n)<=.004 and abs(error@a)<=.020 and abs(error[2])<=.010)

    def base_move(self,delta,expected_arm,label):
        delta=list(map(float,delta));duration=finite_duration(self.profile,delta,max_translation_m=None)
        state=self.state()
        if not pose_matches(state['joints'],list(expected_arm)+[.65]):
            raise ValueError('open_overhead_arm_required')
        target=endpoint(state['poses']['odom'],delta,max_translation_m=None)
        topic=TOPICS[1];pub=self.publisher(topic)
        try:
            self.ownership(topic)
            self.record(dict(event='horizontal_base_intent',label=label,delta=delta,target=target,duration_s=duration))
            pub.publish(self.trajectory(['odom_x','odom_y','odom_t'],target,duration))
            self.record(dict(event='horizontal_base_sent',label=label,delta=delta))
            sent=time.monotonic()
            while time.monotonic()-sent<duration+.8:
                self.pump(.02);self.ownership(topic)
                actual=self.state()
                if not pose_matches(actual['joints'],list(expected_arm)+[.65]):
                    raise ValueError('arm_changed_during_overhead_base_motion')
            return self.state()
        finally:self.node.destroy_publisher(pub)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--continue-open-overhead',type=Path)
    args=p.parse_args()
    if not args.run:
        print(json.dumps(dict(status='horizontal_can_plan_only',robot_commands_sent=False)));return
    if (os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or not is_run_path(args.output.resolve())):
        raise ValueError('isolated_domain71_and_D_output_required')
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    root=args.output;runtime=worker=None;serial=0;report=dict(status='starting',close_commands_sent=0,
        release_commands_sent=0,approach_commands_sent=0,grasp_success_verified=False,
        operator_confirmed_identity=True,model_identity_threshold_changed=False,
        navigation_parameters_changed=False,physical_transfer_verified=False)
    with RobotLock():
        root.mkdir(exist_ok=False);started=time.time_ns()
        with (root/'journal.jsonl').open('x') as journal:
            def record(event):
                if event.get('event')=='can_closure_sent':report['close_commands_sent']+=1
                if event.get('event')=='horizontal_base_sent':report['approach_commands_sent']+=1
                journal.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');journal.flush()
                write_json(root/'result.json',report)
            def stage(name):
                report['stage']=name;write_json(root/'result.json',report)
            try:
                import rclpy
                rclpy.init();epoch='horizontal-grasp-'+uuid.uuid4().hex
                runtime=HorizontalCanRuntime(root,epoch,record)
                runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
                report['initial_state']=state
                continuing=args.continue_open_overhead is not None
                if continuing:
                    previous_path=args.continue_open_overhead.resolve()
                    if not is_run_path(previous_path):
                        raise ValueError('previous_trial_must_be_on_D')
                    previous=json.loads((previous_path/'result.json').read_text())
                    if (previous.get('status')!='stopped_requires_actual_state_review'
                            or previous.get('close_commands_sent')!=0 or previous.get('release_commands_sent')!=0
                            or previous.get('stage')!='align_above_can'
                            or not pose_matches(state['joints'],previous['last_robot_state']['joints'])
                            or not hand_is_open(state['joints'][5])
                            or math.dist(state['poses']['odom'][:2],previous['last_robot_state']['poses']['odom'][:2])>.03):
                        raise ValueError('actual_previous_open_overhead_state_required')
                    report['continued_from_open_overhead']=str(previous_path)
                    # Torso/head height is coupled to lift: re-aim before asking
                    # the head camera to see the now-near, lower target.
                    runtime.face_source((0.,-.8))
                elif not pose_matches(state['joints'][:5]+[0.],[0.,0.,0.,-math.pi/2,0.,0.]):
                    raise ValueError('fresh_empty_tucked_start_required')
                cfg=json.loads((runtime.code/'config/apple_transfer_auto_v1.json').read_text())
                worker=Worker(cfg['model_python'],cfg['registry'],root/'model.json',imgsz=640)
                sample=runtime.sample(processing_reserve_ns=0);capture=runtime.save(sample,'source')
                detections=worker.infer(capture,pump=runtime.pump)
                write_json(capture/'detections.json',detections)
                cans=[d for d in detections['detections'] if d['name']=='canned_juice']
                if len(cans)!=1:raise ValueError('one_detected_operator_confirmed_can_required')
                model=estimate(sample['rgb'],sample['depth'],sample['snapshot'],cans[0])
                appearance=body_appearance(sample['rgb'],sample['depth'],sample['snapshot'],model)
                write_json(root/'source-geometry.json',model);write_json(root/'body-appearance.json',appearance)
                plan=overhead_plan(model);write_json(root/'plan.json',plan)
                # Open and extend while still well outside the target surface.
                stage('prepare_overhead')
                if continuing:
                    plan['goal']=runtime.state()['joints']
                    plan['existing_open_overhead_pose_reused']=True
                    write_json(root/'plan.json',plan)
                else:
                    goal=state['joints'].copy();goal[0]=plan['goal'][0]
                    runtime.move_joints(goal,'raise_outside_table')
                    goal[5]=.65;runtime.move_joints(goal,'open_empty_hand',hand=True)
                    runtime.move_joints(plan['goal'],'orient_top_down_across_can')
                s=runtime.sample(processing_reserve_ns=0);palm=runtime.palm_pose(s['snapshot'])
                predicted,rot=forward(plan['goal'])
                actual_r=rotation(palm['quaternion'])
                write_json(root/'prepared-palm.json',dict(actual=palm,predicted_xyz=predicted.tolist(),predicted_rotation=rot.tolist()))
                if np.linalg.norm(actual_r[:,2]-[0.,0.,-1.])>.05:
                    raise ValueError('actual_fingers_not_pointing_down')
                # Capture the actual wrist sign before choosing the final roll.
                if abs(actual_r[:,1]@np.asarray(model['axis_base']))>.12:
                    raise ValueError('actual_closing_axis_not_across_can')
                runtime.save(s,'prepared-overhead')
                if not continuing:runtime.face_source((0.,-.8))
                report['prepared_state']=runtime.state()
                if args.prepare_only:
                    report['status']='overhead_pose_verified_before_approach';return
                from canned_grasp_geometry import transform_point
                source_s=sample['snapshot'];source_rot=rotation(source_s['tracking_transforms']['odom_from_base_at_depth']['quaternion'])
                center_odom=transform_point(source_s,model['center_base_m'])
                axis_odom=source_rot@np.asarray(model['axis_base'])
                def observe():
                    nonlocal serial,center_odom,axis_odom
                    last=None
                    for _ in range(3):
                        sample_=runtime.sample(processing_reserve_ns=0);snap=sample_['snapshot']
                        expected=transform_point(snap,center_odom,inverse=True)
                        r=rotation(snap['tracking_transforms']['odom_from_base_at_depth']['quaternion'])
                        axis=r.T@axis_odom
                        try:
                            m=measure_near(sample_['rgb'],sample_['depth'],snap,expected,axis,model,appearance)
                            runtime.fresh_measurement(snap);runtime.current_axis=np.asarray(m['axis_base'])
                            center_odom=transform_point(snap,m['center_m']);axis_odom=r@runtime.current_axis
                            serial+=1;path=runtime.save(sample_,f'measure-{serial:03d}',m)
                            record(dict(event='horizontal_body_measured',capture=str(path),measurement=m))
                            return sample_,m
                        except ValueError as exc:last=str(exc)
                    raise ValueError(last or 'horizontal_can_not_observed')
                stage('align_above_can')
                angle=0.
                for index in range(6):
                    before,m=observe();s=before['snapshot'];error=runtime.contact_error(m['center_m'],s)
                    a=runtime.current_axis;n=np.array([-a[1],a[0],0.])
                    if abs(error@n)<=.004 and abs(error@a)<=.020:break
                    desired=error[:2];c,sin=math.cos(angle),math.sin(angle)
                    delta=[c*desired[0]+sin*desired[1],-sin*desired[0]+c*desired[1],0.]
                    runtime.base_move(delta,plan['goal'][:5],f'above-target-{index}')
                    # The hand is above the target: this calibration assumes the
                    # fitted same can remains stationary and uncontacted.
                    after,new=observe();oldtf=s['tracking_transforms']['odom_from_base_at_depth']
                    newtf=after['snapshot']['tracking_transforms']['odom_from_base_at_depth']
                    old_r=rotation(oldtf['quaternion']);new_r=rotation(newtf['quaternion'])
                    odom_step=old_r.T@(np.asarray(newtf['translation'])-oldtf['translation'])
                    actual_step=np.asarray(m['center_m'])-old_r.T@new_r@np.asarray(new['center_m'])
                    d=float(np.linalg.norm(odom_step[:2]));actual_d=float(np.linalg.norm(actual_step[:2]))
                    if d>=.02 and .7<=actual_d/d<=1.3:
                        measured=math.atan2(actual_step[1],actual_step[0])-math.atan2(odom_step[1],odom_step[0])
                        measured=math.atan2(math.sin(measured),math.cos(measured))
                        if abs(measured)<=.35:
                            angle=measured;record(dict(event='stationary_can_motion_direction_observed',angle_rad=angle,
                                assumption='overhead_hand_does_not_contact_can',odom_step=odom_step.tolist(),observed_step=actual_step.tolist()))
                else:raise ValueError('overhead_alignment_not_reached_after_initial_plus_five_corrections')
                stage('descend_to_measured_body')
                before,m=observe();error=runtime.contact_error(m['center_m'],before['snapshot'])
                goal=runtime.state()['joints'];goal[0]+=float(error[2]);runtime.move_joints(goal,'descend_to_can_body_center')
                stage('close_once')
                closure,before,m=runtime.close_gripper(observe,goal[:5])
                holding=float(closure['closure_evidence']['observed_opening_rad'])
                report['holding_opening_rad']=holding
                # Predict the search window from palm motion, then re-fit real
                # body depth after lift. Prediction itself is not lift proof.
                pbefore=runtime.palm_pose(before['snapshot']);rel=rotation(pbefore['quaternion']).T@(np.asarray(m['center_m'])-pbefore['translation'])
                stage('lift_and_verify')
                target=runtime.state()['joints'];target[0]+=.08;target[5]=holding
                runtime.move_joints(target,'lift_horizontal_can')
                rows=[]
                for i in range(3):
                    sample_=runtime.sample(processing_reserve_ns=0);snap=sample_['snapshot'];palm=runtime.palm_pose(snap)
                    predicted=np.asarray(palm['translation'])+rotation(palm['quaternion'])@rel
                    r=rotation(snap['tracking_transforms']['odom_from_base_at_depth']['quaternion'])
                    measured=measure_near(sample_['rgb'],sample_['depth'],snap,predicted,r.T@axis_odom,model,appearance)
                    error=float(np.linalg.norm(np.asarray(measured['center_m'])-predicted))
                    lifted=float(measured['center_m'][2]-m['center_m'][2])
                    path=runtime.save(sample_,f'lift-proof-{i}',measured)
                    row=dict(capture=str(path),follow_error_m=error,measured_lift_m=lifted,
                             opening=runtime.state()['joints'][5]);rows.append(row)
                    if error>.025 or lifted<.03 or abs(row['opening']-holding)>.03:
                        raise ValueError('actual_horizontal_can_not_following_lift')
                report.update(status='horizontal_can_grasp_and_lift_verified',grasp_success_verified=True,
                    lift_evidence=rows,held_center_local_m=rel.tolist(),final_state=runtime.state())
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
                if runtime:
                    try:report['last_robot_state']=runtime.state()
                    except Exception:pass
            finally:
                if worker:worker.close()
                if runtime:runtime.shutdown();rclpy.shutdown()
                report['elapsed_s']=(time.time_ns()-started)/1e9
                write_json(root/'result.json',report);print(json.dumps(report))

if __name__=='__main__':main()

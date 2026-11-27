"""Pepper-specific arm transport over the existing fresh ROS/TF capture runtime.

The short body's grasp height is measured, so the can-only .24-.30m preparation
envelope is not reused. This path stays inside the arm range already exercised
by the existing transfer (.0-.443m), without changing the apple or can entries.
"""
import math
import time
import numpy as np
from canned_grasp_runtime import CanRuntime,NAMES,TOPICS
from canned_grasp_geometry import closure_error,fresh
from hand_camera_ray import rotation
from motion_speed import arm_duration
from task_outcome_policy import pose_matches,hand_is_open
from inverse_table_place_plan import OFFSET,FLEX,WRIST

def arm_goal_with_current_hand(goal,start):
    goal=list(goal);start=list(start)
    if abs(goal[5]-start[5])>.03 and not (hand_is_open(goal[5]) and hand_is_open(start[5])):
        raise ValueError('arm_step_must_preserve_actual_grip')
    goal[5]=start[5]
    return goal

class PepperRuntime(CanRuntime):
    def accept_visual_motion(self,evidence,snapshot):
        angle=evidence.get('controller_to_visual_angle_rad')
        if (type(angle) in (int,float) and math.isfinite(angle) and abs(angle)<=.35
                and evidence.get('reference_epoch')==snapshot['tracking_epoch']
                and evidence.get('current_depth_stamp_ns')==snapshot['streams']['depth']['stamp_ns']):
            self.visual_motion_angle_rad=angle
            self.visual_motion_evidence=evidence

    def controller_delta(self,delta):
        # Only ground_pepper receives this measurement. Head-grasp, apple and
        # can runtimes retain their original numerical trajectory targets.
        angle=getattr(self,'visual_motion_angle_rad',None)
        if angle is None:return list(delta)
        c,s=math.cos(angle),math.sin(angle);x,y,a=delta
        return [c*x+s*y,-s*x+c*y,a]

    def accept_geometry(self,measurement,snapshot):
        # Only the round-shaker adapter uses compact realignment. PenguinRuntime
        # overrides this hook and keeps its existing head/wing approach policy.
        if (measurement.get('class_name')!='ground_pepper'
                or measurement['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']):
            raise ValueError('current_pepper_geometry_required')
        self.current_pepper_geometry=measurement

    def sample(self,timeout=8.,processing_reserve_ns=200_000_000):
        # Leave image processing time without applying that same budget to the
        # separately refreshed joint feedback. RGBD stays at 400ms; this class
        # permits 600ms joint feedback after measuring 412-474ms bridge latency.
        return super().sample(timeout,processing_reserve_ns,joint_processing_reserve_ns=0,
            joint_max_age_ns=600_000_000)

    def _state(self):
        return super()._state(joint_max_age_ns=600_000_000)

    def save(self,sample,name,measurement=None):
        previous=getattr(self,'last_saved_observation',None)
        if (previous is not None and previous[0] is sample and previous[1] is measurement
                and (name=='preclose' or name.startswith('approach-'))):
            self.record(dict(event='reuse_same_observation_capture_before_action',label=name,capture=str(previous[2])))
            return previous[2]
        path=super().save(sample,name,measurement)
        if measurement is not None:(path/'can-measurement.json').rename(path/'object-measurement.json')
        self.last_saved_observation=(sample,measurement,path)
        return path

    def contact_error(self,center,snapshot):return closure_error(center,snapshot)

    def approach_plan(self,error):
        plan=super().approach_plan(error)
        forward,side,height=map(float,error)
        if abs(forward)<=.03 and .004<abs(side)<=.012:
            return dict(phase='small_open_finger_centering',delta=[0.,side,0.])
        measured=getattr(self,'current_pepper_geometry',None)
        if measured is not None and abs(side)>.004:
            # Repeated15cm retreat/advance magnifies odometry/physical-pose
            # disagreement in this Unity noise hierarchy. Keep the full first
            # approach, then re-center the small body at a nearer standoff.
            # This is a positioning target, not a collision certification.
            clearance=max(.06,float(measured['body_radius_m'])+.025)
            if forward<clearance-.01:
                return dict(phase='clear_before_lateral_alignment',delta=[forward-clearance,0.,0.])
            return dict(phase='lateral_alignment',delta=[0.,side,0.])
        return plan

    def fresh_measurement(self,snapshot):
        from canned_carry_geometry import current_images_and_feedback
        state=self.state()
        return current_images_and_feedback(snapshot,state,self.node.get_clock().now().nanoseconds,self.epoch,
            joint_max_age_ns=600_000_000)

    def is_aligned(self,center,snapshot):
        return bool(np.all(abs(self.contact_error(center,snapshot))<=[.020,.004,
            getattr(self,'vertical_alignment_tolerance_m',.006)]))

    def palm_pose(self,snapshot):
        from rclpy.time import Time
        when=Time(nanoseconds=snapshot['streams']['depth']['stamp_ns'])
        return self.tf_dict(self.buf.lookup_transform('base_link','hand_palm_link',when).transform)

    def face_source(self,goal=(0.,-.5)):
        goal=list(goal)
        if len(goal)!=2 or not np.isfinite(goal).all() or not -.6<=goal[0]<=.6 or not -.8<=goal[1]<=0:
            raise ValueError('invalid_source_head_goal')
        sample=self.sample();joints=sample['snapshot']['joints']
        previous=[joints[k] for k in ('head_pan_joint','head_tilt_joint')]
        if (not np.isfinite(previous).all() or not -.65<=previous[0]<=.65
                or not -.85<=previous[1]<=.05):raise ValueError('unexpected_initial_head_pose')
        topic=TOPICS[4];pub=self.publisher(topic);rows=[]
        try:
            self.ownership(topic);pub.publish(self.trajectory(['head_pan_joint','head_tilt_joint'],goal,2.))
            self.record(dict(event='pepper_head_command_sent',goal=goal,duration_s=2.));sent=time.monotonic()
            while time.monotonic()-sent<8:
                sample=self.sample(timeout=3);s=sample['snapshot'];self.ownership(topic)
                positions=[s['joints'][k] for k in ('head_pan_joint','head_tilt_joint')]
                if time.monotonic()-sent<2 or max(abs(a-b) for a,b in zip(positions,goal))>.03:
                    rows=[];continue
                if rows and s['streams']['depth']['stamp_ns']-rows[-1]['depth_stamp_ns']<200_000_000:continue
                rows.append(dict(positions=positions,depth_stamp_ns=s['streams']['depth']['stamp_ns'],joint_stamp_ns=s['joint_stamp_ns']))
                if len(rows)==2:
                    self.record(dict(event='pepper_source_head_view_confirmed',rows=rows));return rows
            raise ValueError('pepper_head_pose_not_observed_after_one_command')
        finally:self.node.destroy_publisher(pub)

    def move_joints(self,goal,label,*,hand=False,before_publish=None):
        goal=list(map(float,goal))
        if (len(goal)!=6 or not np.isfinite(goal).all() or not 0<=goal[0]<=.443
                or not -1.701<=goal[1]<=.001 or abs(goal[2])>.001
                or not -math.pi/2-.001<=goal[3]<=WRIST+.001 or abs(goal[4])>.001
                or not 0<=goal[5]<=.65):raise ValueError('pepper_arm_outside_reviewed_operating_range')
        start=self.state();kind='hand' if hand else 'arm';topic=TOPICS[3 if hand else 2]
        empty_open=not hand and hand_is_open(start['joints'][5])
        if not hand:goal=arm_goal_with_current_hand(goal,start['joints'])
        if hand and not pose_matches(goal[:5]+[0.],start['joints'][:5]+[0.]):raise ValueError('hand_step_must_preserve_arm')
        duration=arm_duration(self.profile,kind,start['joints'],goal,holding=.02<start['joints'][5]<.62,segmented=True)
        pub=self.publisher(topic)
        try:
            if before_publish:before_publish()
            self.ownership(topic);self.record(dict(event='pepper_joint_command_once',label=label,kind=kind,goal=goal,duration_s=duration))
            pub.publish(self.trajectory([NAMES[5]] if hand else NAMES[:5],goal[5:] if hand else goal[:5],duration))
            self.record(dict(event='pepper_joint_command_sent',label=label,kind=kind))
            sent=time.monotonic();stable=None
            while time.monotonic()-sent<duration+5:
                self.pump(.02);state=self.state();self.ownership(topic)
                if not hand and ((empty_open and not hand_is_open(state['joints'][5]))
                        or not empty_open and abs(state['joints'][5]-start['joints'][5])>.03):
                    raise ValueError('pepper_grip_changed_during_arm_step')
                matches=(pose_matches(state['joints'][:5]+[0.],goal[:5]+[0.],endpoint=True)
                    and hand_is_open(state['joints'][5])) if empty_open else pose_matches(state['joints'],goal,endpoint=True)
                if matches:
                    if stable is None:stable=time.monotonic()
                    if time.monotonic()-sent>=duration and time.monotonic()-stable>=.4:
                        return dict(status='passed',goal=goal,final_state=state,duration_s=duration,gripper_command_sent=hand)
                else:stable=None
            raise ValueError('pepper_joint_step_not_observed:'+label)
        finally:self.node.destroy_publisher(pub)

    def prepare(self,lift,body_above_support):
        self.vertical_alignment_tolerance_m=min(.006,body_above_support-.0175-.004)
        if self.vertical_alignment_tolerance_m<=0:raise ValueError('pepper_body_too_low_for_finger_clearance')
        state=self.state();j=state['joints']
        if (not pose_matches(j[:5]+[0.],[0.,0.,0.,-math.pi/2,0.,0.])
                or min(abs(j[5]),abs(j[5]-.65))>.015):raise ValueError('pepper_empty_tucked_start_required')
        target=j.copy();target[0]=lift;self.move_joints(target,'raise_empty_to_measured_body_height')
        if abs(target[5]-.65)>.005:
            target[5]=.65;self.move_joints(target,'open_empty_gripper',hand=True)
        target[1]=FLEX;target[3]=WRIST;self.move_joints(target,'extend_horizontal_at_body_height')
        return target[:5]

    def lift_holding(self,holding):
        target=self.state()['joints'];target[0]+=.1;target[5]=holding
        return self.move_joints(target,'verify_object_lift')

    def retract_holding(self,holding):
        target=self.state()['joints']
        if abs(target[5]-holding)>.03:raise ValueError('pepper_grip_changed_before_retraction')
        target[1]=0.;target[3]=-math.pi/2;self.move_joints(target,'fold_preserving_grip')
        target[0]=0.;self.move_joints(target,'lower_tucked_preserving_grip')

    def prepare_place(self,palm_height,holding):
        target=self.state()['joints']
        if not pose_matches(target,[0.,0.,0.,-math.pi/2,0.,holding]):raise ValueError('pepper_tucked_hold_required_for_placement')
        lift=palm_height-OFFSET
        if not .07<=lift<=.443:raise ValueError('pepper_destination_height_outside_reviewed_arm_range')
        target[0]=.4428021848314039;self.move_joints(target,'raise_tucked_before_placement')
        target[1]=FLEX;target[3]=WRIST;self.move_joints(target,'extend_horizontal_above_table')
        target[0]=lift;self.move_joints(target,'set_measured_release_height')

    def release(self,before_publish):
        target=self.state()['joints'];target[5]=.65
        return self.move_joints(target,'release_once',hand=True,before_publish=before_publish)

    def raise_open(self):
        target=self.state()['joints'];target[0]+=.08
        return self.move_joints(target,'raise_open_hand')

    def tuck_open(self):
        target=self.state()['joints'];target[1]=0.;target[3]=-math.pi/2
        self.move_joints(target,'fold_empty_after_placement');target[0]=0.
        return self.move_joints(target,'lower_empty_after_placement')

    def record_pose(self,label):
        sample=self.sample();self.fresh_measurement(sample['snapshot'])
        return sample,self.save(sample,label)

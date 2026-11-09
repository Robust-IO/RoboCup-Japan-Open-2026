"""Explicit Unity can-grasp transport. No navigation, release or task messages.

Publishers exist only during their single owned finite action. Capture saves
are on demand, not an unbounded image recorder. Original apple code is reused
for preparation/lift without changing its sources or bypassing its close gate.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from canned_grasp_geometry import aligned,closure_error,contact_offset,fresh,measure_near,predicted_before_contact,upright_trial_evidence
from capture_file_writer import write_capture_files
from gripper_close_gate import CloseGate
from motion_speed import duration_fields,finite_duration,load_profile
from reviewed_carry_adjustment import endpoint
from synced_capture_selection import select_stamps
from task_outcome_policy import pose_matches
from can_approach_plan import approach_plan

NAMES=['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']
TOPICS=['/hsrb/'+name for name in ('command_velocity','omni_base_controller/command',
    'arm_trajectory_controller/command','gripper_controller/command','head_trajectory_controller/command')]


class CanRuntime:
    def __init__(self,root,epoch,record,*,extra_seat_m=0.):
        contact_offset(extra_seat_m)
        self.extra_seat_m=extra_seat_m
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image,CameraInfo,JointState
        from tf2_ros import Buffer,TransformListener
        self.rclpy=rclpy;self.root=Path(root);self.epoch=epoch;self.record=record
        self.node=rclpy.create_node('can_grasp_explicit_trial');self.buf=Buffer();self.listener=TransformListener(self.buf,self.node)
        self.bridge=CvBridge();self.pool=ThreadPoolExecutor(max_workers=3);self.received={}
        self.history={k:{} for k in ('rgb','depth','hand')};self.last=None
        self.rejected_serial=0
        self.profile=load_profile('balanced_v1');self.code=Path(__file__).parent
        topics={'rgb':('/hsrb/head_rgbd_sensor/rgb/image_raw',Image),
            'depth':('/hsrb/head_rgbd_sensor/depth_registered/image_raw',Image),
            'hand':('/hsrb/hand_camera/image_raw',Image),
            'rgb_info':('/hsrb/head_rgbd_sensor/rgb/camera_info',CameraInfo),
            'depth_info':('/hsrb/head_rgbd_sensor/depth_registered/camera_info',CameraInfo),
            'hand_info':('/hsrb/hand_camera/camera_info',CameraInfo),'joints':('/hsrb/joint_states',JointState)}
        self.subs=[self.node.create_subscription(kind,topic,lambda msg,key=key:self.receive(key,msg),qos_profile_sensor_data)
            for key,(topic,kind) in topics.items()]

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec

    def receive(self,key,message):
        self.received[key]=message
        if key in self.history:
            rows=self.history[key];rows[self.stamp(message)]=message
            while len(rows)>40:rows.pop(next(iter(rows)))

    def pump(self,seconds=.01):
        self.rclpy.spin_once(self.node,timeout_sec=seconds)
        for _ in range(30):self.rclpy.spin_once(self.node,timeout_sec=0.)

    @staticmethod
    def tf_dict(tf):
        t,q=tf.translation,tf.rotation
        return dict(translation=[t.x,t.y,t.z],quaternion=[q.x,q.y,q.z,q.w])

    def sample(self,timeout=8.,processing_reserve_ns=200_000_000,*,joint_processing_reserve_ns=None,
            joint_max_age_ns=400_000_000):
        from rclpy.time import Time
        deadline=time.monotonic()+timeout;reason='missing_streams'
        def required(stamps):
            rgb_ns,depth_ns,hand_ns=stamps
            rgb=self.history['rgb'][rgb_ns];dep=self.history['depth'][depth_ns];hand=self.history['hand'][hand_ns]
            depth_time=Time(nanoseconds=depth_ns);hand_time=Time(nanoseconds=hand_ns)
            return [('base_link',rgb.header.frame_id,depth_time),(rgb.header.frame_id,dep.header.frame_id,depth_time),
                ('base_link',hand.header.frame_id,hand_time),('hand_palm_link',hand.header.frame_id,hand_time),
                ('odom','base_link',depth_time),('base_link','odom',hand_time),('map','base_link',depth_time),
                ('base_link','hand_palm_link',depth_time)]
        def ready(stamps):return all(self.buf.can_transform(t,s,when) for t,s,when in required(stamps))
        while time.monotonic()<deadline:
            self.pump()
            if len(self.received)!=7:continue
            joint=self.received['joints'];now=self.node.get_clock().now().nanoseconds
            stamps=select_stamps(self.history,now,self.last,ready=ready,
                joint_stamp_ns=self.stamp(joint),processing_reserve_ns=processing_reserve_ns,
                joint_processing_reserve_ns=joint_processing_reserve_ns,joint_max_age_ns=joint_max_age_ns)
            if stamps is None:reason='waiting_for_coherent_image_time_tf';continue
            messages=dict(self.received)
            messages.update({k:self.history[k][ns] for k,ns in zip(('rgb','depth','hand'),stamps)})
            transforms=[self.buf.lookup_transform(t,s,when).transform for t,s,when in required(stamps)]
            report=dict(tracking_epoch=self.epoch,
                streams={k:dict(stamp_ns=self.stamp(m),frame=m.header.frame_id,width=m.width,height=m.height)
                    for k,m in messages.items() if k!='joints'},
                rgb_k=list(messages['rgb_info'].k),rgb_d=list(messages['rgb_info'].d),
                depth_k=list(messages['depth_info'].k),depth_d=list(messages['depth_info'].d),
                hand_k=list(messages['hand_info'].k),hand_d=list(messages['hand_info'].d),
                joints=dict(zip(joint.name,joint.position)),joint_stamp_ns=self.stamp(joint),
                depth_encoding=messages['depth'].encoding,motion_authorized=False,
                palm_base_at_depth=self.tf_dict(transforms[7])['translation'])
            for key,tf in zip(('base_from_rgb','rgb_from_depth','base_from_hand','palm_from_hand'),transforms[:4]):
                report[key]=self.tf_dict(tf)
            report['tracking_transforms']={}
            for index,key in ((4,'odom_from_base_at_depth'),(5,'base_from_odom_at_hand'),(6,'map_from_base_at_depth')):
                target,source,when=required(stamps)[index]
                report['tracking_transforms'][key]=dict(**self.tf_dict(transforms[index]),stamp_ns=when.nanoseconds,
                    source_frame=source,target_frame=target)
            rgb=self.bridge.imgmsg_to_cv2(messages['rgb'],'rgb8')
            hand=self.bridge.imgmsg_to_cv2(messages['hand'],'bgr8')
            depth=self.bridge.imgmsg_to_cv2(messages['depth'],'passthrough').astype(np.float32)
            if report['depth_encoding']=='16UC1':depth*=.001
            elif report['depth_encoding']!='32FC1':raise ValueError('unsupported_can_depth_encoding')
            try:fresh(report,self.node.get_clock().now().nanoseconds,self.epoch,joint_max_age_ns=joint_max_age_ns)
            except ValueError as exc:
                if str(exc)!='fresh_can_rgbd_and_joints_required':raise
                reason='selected_capture_expired_during_decoding_reacquire_new_frame'
                continue
            self.last=stamps
            return dict(rgb=rgb,hand=hand,depth=depth,snapshot=report)
        raise RuntimeError(reason)

    def save(self,sample,name,measurement=None):
        output=self.root/'captures'/name;output.mkdir(parents=True,exist_ok=False)
        write_capture_files(output,sample['rgb'][:,:,::-1].copy(),sample['hand'],sample['depth'],sample['snapshot'],self.pool)
        if measurement is not None:(output/'can-measurement.json').write_text(json.dumps(measurement,indent=2))
        return output

    def observe_near(self,reference,task_id,max_frames=5):
        """Reacquire NEW depth on an unusable frame, never repeat an action."""
        if type(max_frames) is not int or not 1<=max_frames<=5:raise ValueError('invalid_can_reacquisition_window')
        reason='no_current_can_surface'
        for _ in range(max_frames):
            sample=self.sample();snapshot=sample['snapshot']
            expected=predicted_before_contact(reference,snapshot,task_id)
            try:
                if reference.get('require_upright_trial'):
                    measurement=measure_near(sample['depth'],snapshot,expected,reference['radius_m'],
                        reference['height_m'],rgb=sample['rgb'],appearance=reference['source_appearance'])
                    measurement['upright_trial_evidence']=upright_trial_evidence(measurement,reference['maximum_preclose_tilt_deg'])
                    fresh(snapshot,self.node.get_clock().now().nanoseconds,self.epoch)
                    return sample,measurement
                try:
                    measurement=measure_near(sample['depth'],snapshot,expected,reference['radius_m'],reference['height_m'])
                except ValueError as exc:
                    if (str(exc)!='cylinder_radius_outside_observation_envelope'
                            or 'source_appearance' not in reference):raise
                    # A tilted or finger-occluded side is not an upright circle.
                    # Source RGB selects actual cap/side depth; the same radius
                    # bounds and observed-surface checks remain in force.
                    measurement=measure_near(sample['depth'],snapshot,expected,reference['radius_m'],
                        reference['height_m'],rgb=sample['rgb'],appearance=reference['source_appearance'])
                    measurement['precontact_upright_fit_rejection']=str(exc)
                fresh(snapshot,self.node.get_clock().now().nanoseconds,self.epoch)
                return sample,measurement
            except ValueError as exc:
                reason=str(exc)
                path=self.save(sample,f'rejected-geometry-{self.rejected_serial:03d}')
                self.rejected_serial+=1
                self.record(dict(event='discard_depth_frame_no_action_replayed',reason=reason,capture=str(path)))
        raise ValueError('can_geometry_not_resolved_after_new_frames:'+reason)

    def state(self):
        from fresh_feedback_wait import wait_for_fresh
        return wait_for_fresh(self._state,self.pump)

    def _state(self,*,joint_max_age_ns=400_000_000):
        from rclpy.time import Time
        self.pump(0.)
        if 'joints' not in self.received:raise RuntimeError('stale_joints')
        joint=self.received['joints'];now=self.node.get_clock().now().nanoseconds
        if not -20_000_000<=now-self.stamp(joint)<=joint_max_age_ns:raise RuntimeError('stale_joints')
        values=dict(zip(joint.name,joint.position));joints=[float(values[k]) for k in NAMES]
        if not np.isfinite(joints).all():raise RuntimeError('invalid_can_robot_joints')
        poses={}
        for frame in ('map','odom'):
            tf=self.buf.lookup_transform(frame,'base_footprint',Time())
            if not -20_000_000<=now-self.stamp(tf)<=400_000_000:raise RuntimeError('stale_base_tf')
            t,q=tf.transform.translation,tf.transform.rotation
            poses[frame]=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        return dict(joints=joints,poses=poses,stamp_ns=self.stamp(joint))

    def ownership(self,owned=None):
        for topic in TOPICS:
            if self.node.count_publishers(topic)!=(1 if topic==owned else 0):
                raise RuntimeError('other_can_motion_controller:'+topic)
        if owned and self.node.count_subscribers(owned)!=1:raise RuntimeError('can_controller_not_connected')

    def publisher(self,topic):
        from trajectory_msgs.msg import JointTrajectory
        self.ownership();pub=self.node.create_publisher(JointTrajectory,topic,10)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            self.pump()
            if pub.get_subscription_count()==1:
                self.ownership(topic);return pub
        self.node.destroy_publisher(pub);raise RuntimeError('can_command_receiver_unavailable')

    @staticmethod
    def trajectory(names,positions,duration):
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        msg=JointTrajectory();msg.joint_names=names
        point=JointTrajectoryPoint();point.positions=list(map(float,positions))
        point.time_from_start.sec,point.time_from_start.nanosec=duration_fields(duration);msg.points=[point]
        return msg

    def child(self,args,directory):
        args=list(args)
        if (Path(args[0]).name=='pregrasp_raised_trial.py' and '--efficient-transfer' in args
                and '--coherent-can-startup' not in args):
            args.append('--coherent-can-startup')
        self.ownership();path=self.root/directory;log=self.root/(directory+'.log')
        self.record(dict(event='child_intent',directory=directory,arguments=list(map(str,args))))
        with log.open('x') as stream:
            child=subprocess.Popen([sys.executable,*map(str,args),'--output',str(path)],stdout=stream,
                stderr=subprocess.STDOUT,start_new_session=True)
            try:
                while child.poll() is None:self.pump(.03)
            except BaseException:
                from apple_transfer_auto_backend import stop_group
                stop_group(child)
                raise
        report=json.loads((path/'result.json').read_text())
        if child.returncode or report.get('status')!='passed':raise RuntimeError('can_child_failed:'+directory+':'+str(report.get('reason')))
        return report

    def approach(self,observe,expected_arm,index):
        topic=TOPICS[1];pub=self.publisher(topic)
        report=dict(index=index,command_sent=False,collision_warnings_only=True,automatic_retry=False,
            navigation_parameters_changed=False,max_translation_m=None)
        try:
            sample,measurement=observe();snap=sample['snapshot'];state=self.state()
            if not pose_matches(state['joints'],list(expected_arm)+[.65]):raise RuntimeError('can_approach_arm_changed')
            if self.is_aligned(measurement['center_m'],snap):return dict(report,status='already_aligned')
            error=self.contact_error(measurement['center_m'],snap)
            if abs(error[2])>.02:raise RuntimeError('can_approach_height_not_aligned')
            plan=self.approach_plan(error);desired_delta=plan['delta'];delta=self.controller_delta(desired_delta)
            if index==0 and plan['phase']=='straight_contact_approach' and delta[0]<=0:
                raise RuntimeError('can_first_approach_not_forward')
            duration=finite_duration(self.profile,delta,max_translation_m=None)
            target=endpoint(state['poses']['odom'],delta,max_translation_m=None)
            evidence=self.save(sample,f'approach-{index:02d}-before',measurement)
            self.fresh_measurement(snap);self.ownership(topic)
            report.update(phase=plan['phase'],measured_contact_error_m=error.tolist(),
                diagonal_contact_motion=False,delta=delta,start_odom=state['poses']['odom'],target_odom=target,
                duration_s=duration,source_capture=str(evidence))
            if delta!=desired_delta:
                report.update(desired_visual_delta_m=desired_delta,
                    controller_frame_angle_compensation_rad=self.visual_motion_angle_rad,
                    compensation_source='this_task_static_rgbd_motion_not_object_position')
            self.record(dict(event='finite_can_approach_intent',**report))
            pub.publish(self.trajectory(['odom_x','odom_y','odom_t'],target,duration));report['command_sent']=True
            self.record(dict(event='finite_can_approach_sent',index=index,target_odom=target))
            sent=time.monotonic();faults=[];recent=[]
            while time.monotonic()-sent<duration+1.2:
                self.pump(.02)
                try:
                    current=self.state();self.ownership(topic)
                    if not pose_matches(current['joints'],list(expected_arm)+[.65]):raise RuntimeError('can_approach_arm_changed')
                    if time.monotonic()-sent>=duration+.5:recent.append(current['poses']['odom'])
                except Exception as exc:faults.append(str(exc))
            final=self.state()
            if not recent:raise RuntimeError('can_approach_no_final_feedback')
            spread=max(math.dist(p[:2],final['poses']['odom'][:2]) for p in recent)
            if spread>.02:raise RuntimeError('can_base_not_settled')
            fatal=[f for f in faults if 'stale_' not in f and 'extrapolation' not in f.lower()]
            if fatal:raise RuntimeError('can_approach_monitor_fault:'+fatal[-1])
            report.update(status='completed_requires_new_can_measurement',final_state=final,spread_m=spread,
                planned_endpoint_error_m=math.dist(final['poses']['odom'][:2],target[:2]),
                planned_endpoint_is_success_gate=False,monitor_faults=sorted(set(faults)),completed_ns=time.time_ns())
            return report
        finally:self.node.destroy_publisher(pub)

    def close_gripper(self,observe,expected_arm):
        topic=TOPICS[3];pub=self.publisher(topic)
        try:
            sample,measurement=observe();snap=sample['snapshot'];state=self.state()
            if not pose_matches(state['joints'],list(expected_arm)+[.65]) or not self.is_aligned(measurement['center_m'],snap):
                raise RuntimeError('can_not_aligned_no_close_sent')
            before=self.save(sample,'preclose',measurement);duration=6.
            self.fresh_measurement(snap);self.ownership(topic)
            gate=CloseGate(state['joints'][5],duration)
            self.record(dict(event='can_closure_intent',before=str(before),current_depth_measurement=measurement))
            pub.publish(self.trajectory(['hand_motor_joint'],[0.],duration))
            self.record(dict(event='can_closure_sent',before=str(before),duration_s=duration))
            sent=time.monotonic();last=state['stamp_ns'];anchor_arm=state['joints'][:5]
            while time.monotonic()-sent<duration+5:
                self.pump(.02);current=self.state();self.ownership(topic)
                if current['stamp_ns']<=last:continue
                last=current['stamp_ns']
                decision=gate.observe(time.monotonic()-sent,last,current['joints'][5],
                    max(abs(a-b) for a,b in zip(anchor_arm,current['joints'][:5])))
                if decision is None:continue
                result=dict(status='passed' if decision['state']=='contact_candidate' else 'no_contact',
                    grasp_closure_commanded=True,closure_evidence=decision,before_capture=str(before))
                (self.root/'closure-contact.json').write_text(json.dumps(result,indent=2))
                if result['status']!='passed':raise RuntimeError('can_'+decision['state'])
                return result,sample,measurement
            raise RuntimeError('can_closure_feedback_not_stable')
        finally:self.node.destroy_publisher(pub)

    def contact_error(self,center,snapshot):
        return closure_error(center,snapshot,extra_seat_m=self.extra_seat_m)

    def approach_plan(self,error):
        return approach_plan(error)

    def controller_delta(self,delta):return list(delta)

    def fresh_measurement(self,snapshot):
        return fresh(snapshot,self.node.get_clock().now().nanoseconds,self.epoch)

    def is_aligned(self,center,snapshot):
        return aligned(center,snapshot,extra_seat_m=self.extra_seat_m)

    def record_closed_pose(self,reference,task_id):
        """Diagnostic RGBD only; an unsuccessful fit adds no new lift gate."""
        rows=[]
        for index in range(3):
            sample=self.sample();snap=sample['snapshot'];measurement=None
            row=dict(pose_measured=False,diagnostic_only=True,motion_authorized=False)
            try:
                expected=predicted_before_contact(reference,snap,task_id)
                measurement=measure_near(sample['depth'],snap,expected,reference['radius_m'],
                    reference['height_m'],rgb=sample['rgb'],appearance=reference['source_appearance'])
                fresh(snap,self.node.get_clock().now().nanoseconds,self.epoch)
                row.update(pose_measured=True,tilt_deg=measurement['tilt_deg'],center_m=measurement['center_m'])
            except ValueError as exc:
                row['measurement_error']=str(exc);measurement=None
            row['capture']=str(self.save(sample,f'postclose-before-lift-{index:02d}',measurement))
            rows.append(row);self.record(dict(event='postclose_pose_before_lift',**row))
        return rows

    def shutdown(self):
        self.pool.shutdown(wait=True);self.node.destroy_node()

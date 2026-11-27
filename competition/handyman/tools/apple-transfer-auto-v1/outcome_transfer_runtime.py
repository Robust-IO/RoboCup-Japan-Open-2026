"""Outcome policy for the new box adapter. Existing robot limits stay intact."""
import time
from pathlib import Path
from outcome_transfer_policy import load,ClosureEvidence,joint_endpoint_reached,stationary_action_feedback
from horizontal_can_grasp_plan import validate_goal
from pepper_runtime import arm_goal_with_current_hand
from canned_grasp_runtime import NAMES,TOPICS
from motion_speed import arm_duration
from task_outcome_policy import pose_matches

class OutcomeRuntime:
    def repair_sensor_inputs(self,profile):
        """Recreate only stalled read-only subscriptions; never restart Unity/control."""
        from observation_profiles import channels
        now=time.monotonic()
        if not hasattr(self,'_sensor_recovery_since'):
            self._sensor_recovery_since=now;return
        if now-self._sensor_recovery_since<10:return
        self._sensor_recovery_since=now
        if not hasattr(self,'subs') or len(self.subs)!=7:return
        from sensor_msgs.msg import Image,CameraInfo,JointState
        from rclpy.qos import qos_profile_sensor_data
        specs=[('rgb','/hsrb/head_rgbd_sensor/rgb/image_raw',Image),
            ('depth','/hsrb/head_rgbd_sensor/depth_registered/image_raw',Image),
            ('hand','/hsrb/hand_camera/image_raw',Image),
            ('rgb_info','/hsrb/head_rgbd_sensor/rgb/camera_info',CameraInfo),
            ('depth_info','/hsrb/head_rgbd_sensor/depth_registered/camera_info',CameraInfo),
            ('hand_info','/hsrb/hand_camera/camera_info',CameraInfo),('joints','/hsrb/joint_states',JointState)]
        wanted=set(channels(profile));wanted.update(k+'_info' for k in list(wanted));wanted.add('joints')
        repaired=[];missing=[]
        for index,(key,topic,kind) in enumerate(specs):
            if key not in wanted:continue
            if not self.node.count_publishers(topic):missing.append(topic);continue
            self.node.destroy_subscription(self.subs[index])
            self.subs[index]=self.node.create_subscription(kind,topic,lambda msg,key=key:self.receive(key,msg),qos_profile_sensor_data)
            if key in self.history:self.history[key].clear()
            self.received.pop(key,None);repaired.append(topic)
        self.record(dict(event='sensor_subscription_recovery',profile=profile,recreated=repaired,upstream_missing=missing,robot_commands_sent=False))

    def state(self):
        from gate_recovery import transient_observation_error
        while True:
            try:
                state=super().state()
                msg=self.received['joints'];j=dict(zip(msg.name,msg.position))
                observed=dict(state,head=[float(j[n]) for n in ('head_pan_joint','head_tilt_joint')])
                self.observation_cycle().observe(observed,now=time.monotonic())
                return state
            except Exception as exc:
                if not transient_observation_error(exc):raise
                self.recovery_status().waiting(str(exc));self.repair_sensor_inputs('state');self.pump(.05)

    def save(self,sample,name,measurement=None):
        from profiled_capture_writer import save
        return save(self,sample,name,measurement)

    def _observation_metrics(self):
        if not hasattr(self,'_capture_metrics'):self._capture_metrics=dict(acquired={},reused={},invalidations=0)
        return self._capture_metrics

    def shutdown(self):
        import json
        (self.root/'observation-metrics.json').write_text(json.dumps(self._observation_metrics(),indent=2))
        return super().shutdown()

    def table_geometry(self,sample,table,plane):
        from source_table_front import observed_front
        if not hasattr(self,'_live_table_geometry'):self._live_table_geometry={}
        try:
            actual,proof=observed_front(sample['depth'],sample['snapshot'],table,plane)
            self._live_table_geometry[table['id']]=actual
            self.record(dict(event='actual_table_front_geometry',table_id=table['id'],proof=proof,
                localization_is_independently_laser_verified=getattr(self,'_table_localization_verified',False),global_map_changed=False))
        except ValueError as exc:
            self.record(dict(event='table_front_not_visible',table_id=table['id'],reason=str(exc),
                using='previous_actual_table_front' if table['id'] in self._live_table_geometry else 'laser_aligned_imported_footprint'))
        return self._live_table_geometry.get(table['id'],table)

    def observation_cycle(self):
        if not hasattr(self,'_observation_cycle'):
            from stop_observe_act import ObservationCycle
            self._observation_cycle=ObservationCycle(load()['feedback'])
        return self._observation_cycle

    def observation_state(self):
        state=self.state();msg=self.received['joints'];j=dict(zip(msg.name,msg.position))
        state['head']=[float(j[n]) for n in ('head_pan_joint','head_tilt_joint')]
        return state

    def invalidate_observations(self,*,duration=0.,map_only=False):
        if map_only:self.observation_cycle().invalidate_map(capture_after_ns=self.node.get_clock().now().nanoseconds)
        else:self.observation_cycle().invalidate(now=time.monotonic(),duration=duration)
        self._observation_metrics()['invalidations']+=1
        if hasattr(self,'_observation_cache'):self._observation_cache.clear()

    def trajectory(self,names,positions,duration):
        self.invalidate_observations(duration=duration)
        return super().trajectory(names,positions,duration)

    def _acquire_observation(self,profile,**kwargs):
        from profiled_capture import acquire
        return acquire(self,profile,**kwargs)

    def recovery_status(self):
        if not hasattr(self,'_gate_recovery'):
            from gate_recovery import RecoveryStatus
            self._gate_recovery=RecoveryStatus(self.root,self.record)
        return self._gate_recovery

    def sample(self,timeout=None,processing_reserve_ns=0,*,profile=None,force_new=False):
        from observation_profiles import channels
        from gate_recovery import transient_observation_error
        profile=profile or getattr(self,'default_observation_profile','rgbd');channels(profile)
        cycle=self.observation_cycle();recovery=self.recovery_status()
        # timeout is a legacy call argument, not an overall task deadline.
        # Polling remains bounded so callbacks, recovery notices and cancellation work.
        while True:
            try:
                state=self.observation_state()
                if not cycle.observe(state,now=time.monotonic()):
                    recovery.waiting('waiting_for_robot_to_stop');self.pump(.02);continue
                cutoff=max(cycle.stop_stamp_ns,cycle.capture_not_before_ns)
                sample=self._acquire_observation(profile,timeout=1.,processing_reserve_ns=0,capture_after_ns=cutoff)
                sample['snapshot']['observation_profile']=profile
                sample['snapshot']['stop_observe_act']=cycle.capture_marker(sample['snapshot'])
                self.fresh_measurement(sample['snapshot'])
                metrics=self._observation_metrics();metrics['acquired'][profile]=metrics['acquired'].get(profile,0)+1
                self._sensor_recovery_since=time.monotonic();recovery.recovered();return sample
            except Exception as exc:
                if not transient_observation_error(exc):raise
                recovery.waiting(str(exc));self.repair_sensor_inputs(profile);self.pump(.05)

    def fresh_measurement(self,snapshot):
        return stationary_action_feedback(snapshot,self.observation_state(),self.node.get_clock().now().nanoseconds,self.epoch,
            current_generation=self.observation_cycle().generation)

    def palm_pose(self,snapshot):
        if 'base_from_palm_at_depth' in snapshot:return snapshot['base_from_palm_at_depth']
        if 'base_from_palm_at_hand' in snapshot:return snapshot['base_from_palm_at_hand']
        return super().palm_pose(snapshot)

    def prepare_navigation_from_current(self):
        # Preserve actual grip; neither an exact initial pose nor an empty hand is required.
        self.sample(profile='state');goal=self.state()['joints'].copy()
        goal[:5]=[0.,0.,0.,-__import__('math').pi/2,0.]
        self.move_joints(goal,'fold_and_lower_current_arm_for_navigation')
        return self.state()

    def move_joints(self,goal,label,*,hand=False,before_publish=None):
        goal=validate_goal(goal);start=self.state();topic=TOPICS[3 if hand else 2]
        if not hand:goal=arm_goal_with_current_hand(goal,start['joints'])
        if hand and not pose_matches(goal[:5]+[0.],start['joints'][:5]+[0.]):raise ValueError('hand_step_must_preserve_arm')
        def reached(actual):
            return joint_endpoint_reached(actual,goal,hand=hand)
        if reached(start['joints']):
            if before_publish:before_publish()
            self.record(dict(event='joint_move_skipped_already_satisfied',label=label,goal=goal))
            return dict(final_state=self.state(),goal=goal,duration_s=0.,command_skipped=True)
        duration=arm_duration(self.profile,'hand' if hand else 'arm',start['joints'],goal,holding=.02<start['joints'][5]<.62,segmented=True)
        if hand and label=='prepare_initial_empty_gripper':
            expected=[0.,0.,0.,-math.pi/2,0.]
            if any(abs(x-y)>.05 for x,y in zip(start['joints'][:5],expected)) or abs(goal[5]-.65)>.001:
                raise ValueError('fast_startup_requires_tucked_open_goal')
            duration=1.5
        pub=self.publisher(topic)
        try:
            if before_publish:before_publish()
            self.ownership(topic);self.record(dict(event='horizontal_joint_intent',label=label,goal=goal,duration_s=duration))
            pub.publish(self.trajectory([NAMES[5]] if hand else NAMES[:5],goal[5:] if hand else goal[:5],duration))
            self.record(dict(event='horizontal_joint_sent',label=label,goal=goal));sent=time.monotonic();stable=None
            while time.monotonic()-sent<duration+5:
                self.pump(.02);actual=self.state();self.ownership(topic)
                if not hand and abs(actual['joints'][5]-start['joints'][5])>load()['hold']['opening_drift_rad']:raise ValueError('grip_changed_during_arm_motion')
                if reached(actual['joints']):
                    stable=time.monotonic() if stable is None else stable
                    if time.monotonic()-sent>=duration and time.monotonic()-stable>=load()['feedback']['arm_settle_s']:
                        return dict(final_state=actual,goal=goal,duration_s=duration)
                else:stable=None
            raise ValueError('joint_endpoint_not_observed:'+label)
        finally:self.node.destroy_publisher(pub)

    def close_gripper(self,observe,expected_arm):
        topic=TOPICS[3];pub=self.publisher(topic)
        try:
            while True:
                sample,m=observe();s=sample['snapshot'];state=self.state()
                if not pose_matches(state['joints'],list(expected_arm)+[.65]) or not self.is_aligned(m['center_m'],s):raise ValueError('not_aligned_no_close_sent')
                s['precise_publish_check']=True
                try:self.fresh_measurement(s)
                except ValueError as exc:
                    from gate_recovery import transient_observation_error
                    if not transient_observation_error(exc):raise
                    self.recovery_status().waiting(str(exc));continue
                break
            before=self.save(sample,'preclose',m);self.ownership(topic)
            gate=ClosureEvidence(state['joints'][5]);self.record(dict(event='can_closure_intent',before=str(before),current_depth_measurement=m))
            pub.publish(self.trajectory(['hand_motor_joint'],[0.],6.));self.record(dict(event='can_closure_sent',before=str(before),duration_s=6.))
            sent=time.monotonic()
            while time.monotonic()-sent<11:
                self.pump(.02);current=self.state();self.ownership(topic)
                d=gate.observe(time.monotonic()-sent,current['stamp_ns'],current['joints'][5],max(abs(a-b) for a,b in zip(state['joints'][:5],current['joints'][:5])))
                if d is None:continue
                result=dict(status='passed' if d['state']=='contact_candidate' else 'no_contact',closure_evidence=d,before_capture=str(before),grasp_closure_commanded=True)
                (self.root/'closure-contact.json').write_text(__import__('json').dumps(result,indent=2))
                if result['status']!='passed':raise ValueError('closed_without_contact_evidence')
                return result,sample,m
            raise ValueError('closure_feedback_not_stable')
        finally:self.node.destroy_publisher(pub)

from apple_transfer_auto_backend import LiveBackend
class OutcomeBackend(LiveBackend):
    def command(self,*args,**kwargs):
        runtime=getattr(self,'observation_runtime',None)
        if runtime is not None:runtime.invalidate_observations()
        try:return super().command(*args,**kwargs)
        finally:
            if runtime is not None:runtime.invalidate_observations()

    def adjust(self,name,pose,delta,arm,opening,*,single_placement_approach=False):
        path=self.output/(name+'.json')
        args=[Path(__file__).with_name('outcome_finite_adjustment.py'),'--run','--simulation-ignore-collision',
            '--reviewed-map-pose',*pose,'--delta',*delta,'--expected-arm',*arm,'--holding-opening',opening,
            '--motion-profile',self.config.get('motion_profile','balanced_v1'),'--joint-feedback-max-age-s',.6,
            '--efficient-transfer','--output',path]
        self.command(args,path,'status','endpoint_verified',None)
        return str(path)

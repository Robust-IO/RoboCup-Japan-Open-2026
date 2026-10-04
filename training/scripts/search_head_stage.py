"""Opt-in head stage: isolated domain 73, or explicitly allowed live domain 71.

Runtime owns the one finite trajectory; observer independently verifies joints.
Proof file is private to a task/point directory, never reused across sessions.
"""
import json
import math
import time
from head_view_trial import Settling,JOINTS,COMMAND,target,MAX_FUTURE_SKEW_NS,SEARCH_FEEDBACK_MAX_S


def validate_environment(domain,localhost,allow_live):
    if localhost!='1' or domain!=('71' if allow_live else '73'):
        raise ValueError('head_stage_environment_not_authorized')


def proof_valid(row,task,point,tilt,now_ns):
    return (isinstance(row,dict) and row.get('task_id')==task and row.get('point_id')==point
            and row.get('positions')==[0.,tilt] and row.get('state')=='ready'
            and type(row.get('stamp_ns')) is int
            and -MAX_FUTURE_SKEW_NS<=now_ns-row['stamp_ns']<=1_000_000_000)


class StopConfirmation:
    """Fresh post-hold feedback, never elapsed-time-only completion."""
    def __init__(self,pose,stamp_ns,now):
        self.gate=Settling(pose,SEARCH_FEEDBACK_MAX_S)
        # Even the earliest possible acquisition time must be AFTER the hold.
        self.cutoff=stamp_ns+MAX_FUTURE_SKEW_NS;self.started=now;self.latest=None

    def sample(self,names,positions,stamp_ns,sensor_now_ns,now):
        if stamp_ns<=self.cutoff:
            self.latest=None;self.gate.reset();return
        ok,pose=self.gate.sample(names,positions,stamp_ns,sensor_now_ns,now)
        self.latest=(now,stamp_ns,ok,pose)

    def verified(self,now):
        return (self.latest is not None and self.latest[2] and
                0<=now-self.latest[0]<=SEARCH_FEEDBACK_MAX_S and now-self.started>=.2)


class HeadStage:
    def __init__(self,node,task,point,tilt,path,*,allow_live=False):
        import os
        validate_environment(os.environ.get('ROS_DOMAIN_ID'),os.environ.get('ROS_LOCALHOST_ONLY'),allow_live)
        from trajectory_msgs.msg import JointTrajectory
        from sensor_msgs.msg import JointState
        from rclpy.qos import qos_profile_sensor_data
        self.node=node;self.task=task;self.point=point;self.goal=target(0.,tilt);self.path=path
        self.gate=Settling(self.goal,SEARCH_FEEDBACK_MAX_S);self.latest=None;self.sent=None;self.armed=None
        self.ready=False;self.failed=None;self.cancelled=False
        self.stop=None;self.stop_recorded=False
        self.trace_count=0;self.previous_receive=None;self.last_wait=None
        self.trace_error=None
        self.pub=node.create_publisher(JointTrajectory,COMMAND,10)
        self.sub=node.create_subscription(JointState,'/hsrb/joint_states',self.receive,qos_profile_sensor_data)

    def trace(self,event,**details):
        # Bound high-rate feedback only; lifecycle/fault records must survive the cap.
        if event=='feedback':
            if self.trace_count>=2000:return
            self.trace_count+=1
        row=dict(event=event,monotonic_s=time.monotonic(),task_id=self.task,
                 point_id=self.point,**details)
        try:
            with self.path.with_name('head-trace.jsonl').open('a') as stream:
                stream.write(json.dumps(row,allow_nan=False)+'\n')
        except OSError as exc:
            # Diagnostics must not interrupt a hold/cancel or interface disposal.
            if self.trace_error is None:
                self.trace_error=str(exc)
                self.node.get_logger().error('Head trace unavailable: '+str(exc))

    def snapshot(self):
        now=time.monotonic()
        return dict(reason=self.failed,feedback_reason=self.gate.reason,
                    command_sent=self.sent is not None,ready=self.ready,
                    feedback_age_s=None if self.latest is None else now-self.latest[0],
                    positions=None if self.latest is None else list(self.latest[2]),
                    command_subscribers=self.pub.get_subscription_count(),
                    command_publishers=self.node.count_publishers(COMMAND),
                    hold_sent=self.stop is not None,trace_error=self.trace_error)

    def fail(self,reason):
        if self.failed is None:
            self.failed=reason
            self.trace('head_fault',**self.snapshot())

    def wait(self,reason):
        if reason!=self.last_wait:
            self.trace('head_wait',wait_reason=reason,**self.snapshot())
            self.last_wait=reason

    def receive(self,msg):
        now=time.monotonic();ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        sensor_now=self.node.get_clock().now().nanoseconds
        ok,pose=self.gate.sample(list(msg.name),list(msg.position),ns,sensor_now,now)
        self.latest=(now,ns,pose,ok) if pose is not None else None
        self.trace('feedback',stamp_ns=ns,age_s=(sensor_now-ns)/1e9,
                   receive_gap_s=None if self.previous_receive is None else now-self.previous_receive,
                   positions=pose,raw_names=list(msg.name),
                   raw_positions=[v if math.isfinite(v) else None for v in msg.position],
                   feedback_reason=self.gate.reason,settled=ok,cancelled=self.cancelled)
        self.previous_receive=now
        if self.cancelled:
            self.try_hold()
            if self.stop is not None:
                self.stop.sample(list(msg.name),list(msg.position),ns,self.node.get_clock().now().nanoseconds,now)
            return
        if self.failed:return
        if self.ready and not ok:
            self.fail('head_feedback_invalid_after_ready' if pose is None or self.gate.reason=='feedback_gap'
                      else 'head_changed_after_ready')
        if self.sent is not None and now>=self.sent+2. and ok:
            # A task-scoped, feedback-backed readiness lease, not a one-shot
            # historical certificate. TF reacquisition must see recent evidence.
            # Cancelled/failed paths above never renew it; no motion is resent.
            row=dict(task_id=self.task,point_id=self.point,positions=list(self.goal),state='ready',stamp_ns=ns)
            temporary=self.path.with_suffix('.tmp');temporary.write_text(json.dumps(row));temporary.replace(self.path)
            if not self.ready:
                self.ready=True
                self.trace('head_ready',stamp_ns=ns,positions=pose)

    def tick(self,arrived):
        if self.cancelled or self.failed:return
        now=time.monotonic()
        if arrived and self.armed is None:
            self.armed=now;self.trace('head_armed')
        if self.armed is None:return
        if now-self.armed>8 and not self.ready:self.fail('head_stage_timeout');return
        if self.ready:
            if self.latest is None or now-self.latest[0]>SEARCH_FEEDBACK_MAX_S:self.fail('head_feedback_timeout')
            return
        if self.sent is not None:return
        if self.latest is None or now-self.latest[0]>SEARCH_FEEDBACK_MAX_S:
            self.wait('fresh_feedback_required');return
        if self.pub.get_subscription_count()!=1 or self.node.count_publishers(COMMAND)!=1:
            self.wait('exclusive_command_channel_required');return
        pose=self.latest[2]
        if not (-.65<=pose[0]<=.65 and -.85<=pose[1]<=.05):
            self.fail('head_initial_pose_invalid');return
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        msg=JointTrajectory();msg.joint_names=list(JOINTS)
        p=JointTrajectoryPoint();p.positions=list(self.goal);p.time_from_start.sec=2;msg.points=[p]
        self.pub.publish(msg);self.sent=now;self.gate.reset()
        self.trace('head_command_sent',positions=list(self.goal),previous=list(pose),duration_s=2.)

    def try_hold(self):
        if self.sent is None or self.stop is not None or self.latest is None:return
        now=time.monotonic()
        if (now-self.latest[0]>SEARCH_FEEDBACK_MAX_S or self.pub.get_subscription_count()!=1
                or self.node.count_publishers(COMMAND)!=1):
            self.wait('hold_requires_fresh_feedback_and_exclusive_channel');return
        pose=self.latest[2]
        try:target(*pose)
        except ValueError:
            self.wait('hold_pose_outside_conservative_range');return
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        msg=JointTrajectory();msg.joint_names=list(JOINTS)
        p=JointTrajectoryPoint();p.positions=list(pose);p.time_from_start.nanosec=200_000_000;msg.points=[p]
        self.pub.publish(msg)
        self.stop=StopConfirmation(pose,self.node.get_clock().now().nanoseconds,now)
        self.trace('head_hold_sent',positions=list(pose),duration_s=.2)

    def cancel(self):
        if not self.cancelled:self.trace('head_cancel',**self.snapshot())
        self.cancelled=True
        self.try_hold()

    def drained(self):
        if self.sent is None:return True
        if not self.cancelled or self.stop is None or not self.stop.verified(time.monotonic()):return False
        if not self.stop_recorded:
            self.path.with_name('head-stop.json').write_text(json.dumps(dict(task_id=self.task,point_id=self.point,
                state='stop_confirmed',positions=list(self.stop.latest[3]),stamp_ns=self.stop.latest[1],
                confirmation='post_hold_joint_feedback',hold_duration_s=.2,stationary_window_s=.6)))
            self.stop_recorded=True
            self.trace('head_stop_confirmed',positions=list(self.stop.latest[3]))
        return True

    def close(self):
        # Disposal never initiates a late control command.
        self.trace('head_closed',stop_recorded=self.stop_recorded,**self.snapshot())
        self.cancelled=True;self.node.destroy_subscription(self.sub);self.node.destroy_publisher(self.pub)

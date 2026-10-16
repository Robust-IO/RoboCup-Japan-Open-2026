"""Bounded read-only ROS search observer. Never sends or cancels a goal.

Operator must supply the UUID and pose of the FINAL search-point goal.
Status alone does not contain its pose; this binding is not auto-discovered.
"""
import argparse
import json
import math
from pathlib import Path
import time
import uuid
from search_arrival import SearchArrival
from search_scan import SearchScan
from tf_motion_gate import TfSearchObserver
from probe_tf_motion import planar_pose
from search_goal_binding import SearchGoalBinding
from search_view_evidence import ViewEvidence


def matches_task_cancel(event,row,task_id):
    # A delayed cancellation still applies to its unique task; never revive it.
    return (event=='search_cancelled' and isinstance(row,dict)
            and row.get('schema')=='handyman-search-request-v1'
            and isinstance(task_id,str) and bool(task_id) and row.get('task_id')==task_id)


def cancel_observation(scan,arrival,tf_observer,retries):
    scan.cancel()
    arrival.reset();arrival.adapter=None
    tf_observer.adapter.motion.reset()
    retries.pending=None;retries.waiting=False
    return dict(scan.status(),observer_reason='matching_task_cancelled')


def validate_binding(row, args, sensor_now_ns):
    if not isinstance(row,dict) or row.get('schema')!='handyman-search-binding-v1':
        raise ValueError('invalid_binding_schema')
    stamp=row.get('stamp_ns')
    if type(stamp) is not int or not 0 <= sensor_now_ns-stamp <= 2_000_000_000:
        raise ValueError('stale_binding')
    gate=SearchGoalBinding(args.task_id,args.point_id,dict(x=args.x,y=args.y,yaw=args.yaw),args.map_sha256)
    gate.begin_attempt()
    # Executor generation is recorded for diagnosis; request identity pins this run.
    if type(row.get('generation')) is not int or row['generation']<=0:
        raise ValueError('invalid_generation')
    result=gate.accept(generation=1,task_id=row.get('task_id'),point_id=row.get('point_id'),
        pose=row.get('pose'),map_sha256=row.get('map_sha256'),goal_id=row.get('goal_id'),
        role=row.get('role'),frame_id=row.get('frame_id'))
    result['executor_generation']=row['generation']
    return result


class RetryBindings:
    """Only a confirmed ABORTED goal permits a later binding; cancellation is terminal."""
    def __init__(self, maximum):
        self.maximum=maximum
        self.seen=set()
        self.generation=0
        self.waiting=True
        self.pending=None

    def defer(self,binding,message,now):
        if (self.waiting or len(self.seen)>=self.maximum or binding['goal_id'] in self.seen
                or binding['executor_generation']<=self.generation):
            raise ValueError('invalid_pending_binding')
        if self.pending is not None:
            # Do not refresh lifetime or replace an unresolved candidate.
            if self.pending[0]!=binding:
                raise ValueError('conflicting_pending_binding')
            return
        self.pending=(dict(binding),message,now)

    def take_pending(self,now):
        item=self.pending
        self.pending=None
        if self.waiting and item is not None and 0<=now-item[2]<=2.:
            return item[1]
        return None

    def accept(self,binding):
        key=binding['goal_id']; generation=binding['executor_generation']
        if not self.waiting or len(self.seen)>=self.maximum or key in self.seen or generation<=self.generation:
            raise ValueError('old_duplicate_or_unexpected_retry_binding')
        self.seen.add(key);self.generation=generation;self.waiting=False

    def failed(self,status):
        self.waiting=status==6 and len(self.seen)<self.maximum  # GoalStatus.STATUS_ABORTED
        if not self.waiting:self.pending=None
        return self.waiting


class GoalStatusGate:
    def __init__(self, goal_id):
        self.goal_id = uuid.UUID(goal_id).hex
        self.active = False
        self.done = False

    def consume(self, goal_id, status):
        if goal_id != self.goal_id or self.done:
            return None
        if status in (1, 2):
            self.active = True
        elif status in (4, 5, 6) and self.active:
            self.done = True
            return status == 4
        return None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--goal-id')
    source.add_argument('--binding-topic')
    p.add_argument('--task-id')
    p.add_argument('--point-id')
    p.add_argument('--map-sha256')
    p.add_argument('--cancel-topic',help='Task cancellation topic; requires a unique task-id')
    p.add_argument('--max-bound-attempts',type=int,default=1)
    for name in ('x', 'y', 'yaw'):
        p.add_argument('--'+name, type=float, required=True)
    p.add_argument('--target', default='canned_juice')
    p.add_argument('--target-room')
    p.add_argument('--room-regions',help='JSON room polygons in map coordinates, from bound environment')
    p.add_argument('--action-status-topic',default='/navigate_to_pose/_action/status')
    p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--motion-source', choices=('odom','tf'), default='odom')
    p.add_argument('--recover-transient-tf',action='store_true',
                   help='Allow one bounded re-settling after a TF gap; never renew the search deadline')
    p.add_argument('--seconds', type=float, default=30)
    p.add_argument('--session-budget-mode',action='store_true',help='seconds is remaining session time, not navigation test timeout')
    p.add_argument('--view-seconds',type=float,help='Optional separate observation deadline, shorter than overall deadline')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--vision-topic',default='/handyman/vision/diagnostics')
    p.add_argument('--head-proof',type=Path,help='Private runtime head stage proof')
    p.add_argument('--head-tilt',type=float)
    a = p.parse_args()
    room_gate=None
    if a.target_room or a.room_regions:
        from search_room_gate import RoomGate
        try:room_gate=RoomGate(json.loads(a.room_regions),a.target_room)
        except (ValueError,TypeError,AttributeError):p.error('invalid room regions or target room')
    if a.head_proof is not None:
        import os
        from head_view_trial import target
        if (os.environ.get('ROS_DOMAIN_ID') not in ('71','73') or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
                or a.head_tilt is None or not a.task_id or not a.point_id):
            p.error('head proof requires domain 71/73 localhost, task/point identity and tilt')
        target(0.,a.head_tilt)
    if not 0 < a.seconds <= (600 if a.session_budget_mode else 120):
        p.error('invalid observer/session time budget')
    if a.view_seconds is not None and not 0<a.view_seconds<a.seconds:
        p.error('view-seconds must be positive and shorter than seconds')
    if a.binding_topic and not all((a.task_id,a.point_id,a.map_sha256)):
        p.error('binding requires task-id, point-id and map-sha256')
    if a.cancel_topic:
        try:
            if not a.task_id or uuid.UUID(a.task_id).hex!=a.task_id or a.task_id=='0'*32:
                raise ValueError()
        except (ValueError,AttributeError):p.error('cancel-topic requires a nonzero 32-hex task-id')
    if not 1<=a.max_bound_attempts<=5 or (a.max_bound_attempts>1 and not a.binding_topic):
        p.error('max-bound-attempts must be 1..5; retries require binding-topic')
    retry_bindings=RetryBindings(a.max_bound_attempts)
    waiting_retry=False
    status_gate = GoalStatusGate(a.goal_id or '0'*32)
    bound = not bool(a.binding_topic)
    status_cache = []
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from action_msgs.msg import GoalStatusArray
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException
    start = time.monotonic()
    scan = SearchScan(a.task_id or 'ros-observer', a.target, [dict(x=a.x,y=a.y,yaw=a.yaw)], start,
                      timeout_s=a.seconds, view_s=a.view_seconds or a.seconds)
    evidence=ViewEvidence()
    scan.evidence_ready=lambda now:evidence.snapshot(now)['view_data_usable']
    arrival = SearchArrival(scan, status_gate.goal_id)
    tf_observer = TfSearchObserver(arrival, start, recover_transient_tf=a.recover_transient_tf, max_gap_s=.5)
    # Exclusive log creation preserves prior runs.
    stream = a.output.open('x')
    rclpy.init()
    node = Node('handyman_search_arrival_readonly')
    buffer = Buffer()
    listener = TransformListener(buffer,node)
    def room_check(point,stamp_ns):
        try:
            tf=buffer.lookup_transform('map','odom',Time(nanoseconds=stamp_ns))
            t,q=tf.transform.translation,tf.transform.rotation
            return room_gate.classify(point,[t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
        except (TransformException,ValueError,TypeError):
            return dict(state='transform_unavailable',target_room=a.target_room)
    if room_gate is not None:scan.room_check=room_check
    last_pose = None
    subs = []
    head_gate=None;head_latest=None;head_open=False
    if a.head_proof is not None:
        from head_view_trial import Settling,SEARCH_FEEDBACK_MAX_S
        from sensor_msgs.msg import JointState
        head_gate=Settling((0.,a.head_tilt),SEARCH_FEEDBACK_MAX_S)

        def head_cb(msg):
            nonlocal head_latest
            ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            now=time.monotonic()
            ok,_=head_gate.sample(list(msg.name),list(msg.position),ns,node.get_clock().now().nanoseconds,now)
            head_latest=(now,ns,ok)
        subs.append(node.create_subscription(JointState,'/hsrb/joint_states',head_cb,qos_profile_sensor_data))

    def emit(value,terminal=False):
        nonlocal head_open
        value['monotonic_s']=time.monotonic()
        value['tf_receive_gap_s']=max(0.,time.monotonic()-tf_observer.last_valid)
        value['tf_recovery_used']=tf_observer.recovery_used
        if value.get('arrival_reason')=='arrived_and_settled':
            evidence.arrived(value.get('arrival_stamp_ns'))
            if head_gate is not None:
                head_open=False
                # Defer only the per-view timer. The overall deadline is fixed.
                scan.view_start=scan.deadline
        if value.get('state')=='navigate':evidence.reset()
        if terminal:value['view_evidence']=evidence.snapshot(time.monotonic())
        value.update(actionable=False, read_only=True)
        value.update(terminal=terminal,observer_context=dict(task_id=a.task_id,point_id=a.point_id,map_sha256=a.map_sha256))
        if value.get('state')=='found' and scan.position is not None:
            value.update(position_m=list(scan.position),position_frame='odom')
            if room_gate is not None:value['room_membership']=scan.room_membership
        line = json.dumps(value,allow_nan=False)
        stream.write(line+'\n'); stream.flush()
        print(line,flush=True)

    def invalidate(reason):
        evidence.reset()
        arrival.reset()
        if scan.phase == 'observe':
            scan.phase = 'incomplete'
            scan.position = None
            scan.failures.append(reason)
        emit(dict(scan.status(),observer_reason=reason))

    def status_cb(msg):
        nonlocal bound,waiting_retry,last_pose
        if not bound:
            status_cache.append(msg)
            del status_cache[:-20]
            return
        for item in msg.status_list:
            result = status_gate.consume(bytes(item.goal_info.goal_id.uuid).hex(),item.status)
            if result is not None:
                if result:
                    retry_bindings.pending=None
                    arrival.result(status_gate.goal_id,True,node.get_clock().now().nanoseconds)
                    emit(dict(scan.status(),observer_reason='matching_goal_succeeded'))
                else:
                    scan.phase = 'incomplete'
                    scan.failures.append('navigation_failed_or_cancelled')
                    emit(scan.status())
                    if a.binding_topic and retry_bindings.failed(item.status):
                        waiting_retry=True
                        bound=False
                        last_pose=None
                        arrival.reset()
                        tf_observer.adapter.motion.reset()
                        emit(dict(scan.status(),observer_reason='waiting_for_new_retry_binding'))
                        status_cache.append(msg)
                        del status_cache[:-20]
                        pending=retry_bindings.take_pending(time.monotonic())
                        if pending is not None:
                            binding_cb(pending)
                        return

    def odom_cb(msg):
        nonlocal last_pose
        if not bound:
            return
        try:
            stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            sensor_now = node.get_clock().now().nanoseconds
            if msg.child_frame_id != 'base_footprint' or not 0 <= sensor_now-stamp <= 500_000_000:
                invalidate('invalid_or_stale_odometry'); return
            if last_pose is not None and stamp <= last_pose[1]:
                invalidate('duplicate_or_reversed_odometry'); return
            tf = buffer.lookup_transform('map','base_footprint',Time(nanoseconds=stamp))
            t,q = tf.transform.translation,tf.transform.rotation
            norm = sum(v*v for v in (q.x,q.y,q.z,q.w))
            if not math.isfinite(norm) or abs(norm-1) > .002:
                invalidate('invalid_tf_rotation'); return
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            linear = math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
            angular = abs(msg.twist.twist.angular.z)
            if not all(math.isfinite(v) for v in (t.x,t.y,yaw,linear,angular)):
                invalidate('nonfinite_odometry'); return
            now = time.monotonic()
            if scan.phase == 'observe':
                ax,ay,ayaw = arrival.anchor
                delta = abs(math.atan2(math.sin(yaw-ayaw),math.cos(yaw-ayaw)))
                if linear > .02 or angular > .03 or math.hypot(t.x-ax,t.y-ay) > .02 or delta > .03:
                    invalidate('movement_during_observation'); return
            elif scan.phase == 'navigate':
                emit(arrival.sample(pose=dict(x=t.x,y=t.y,yaw=yaw),linear_speed=linear,
                    angular_speed=angular,stamp_ns=stamp,sensor_now_ns=sensor_now,now=now))
            last_pose = (now,stamp)
        except TransformException:
            invalidate('exact_stamp_tf_unavailable')

    def vision_cb(msg):
        nonlocal head_open
        if scan.phase != 'observe':
            return
        if a.motion_source == 'tf':
            failure = tf_observer.tick(time.monotonic())
            if failure is not None:
                emit(failure); return
        elif last_pose is None or time.monotonic()-last_pose[0] > .5:
            invalidate('odometry_stream_timeout'); return
        if head_gate is not None:
            now=time.monotonic()
            if head_latest is None or not head_latest[2] or now-head_latest[0]>SEARCH_FEEDBACK_MAX_S:
                if head_open:invalidate('head_not_stable_during_observation')
                return
            if not head_open:
                if not a.head_proof.exists():return
                from search_head_stage import proof_valid
                try:
                    proof=json.loads(a.head_proof.read_text())
                except (OSError,ValueError):invalidate('invalid_head_proof');return
                if not proof_valid(proof,a.task_id,a.point_id,a.head_tilt,node.get_clock().now().nanoseconds):
                    invalidate('invalid_head_proof');return
                from search_vision_adapter import SearchVisionAdapter
                cutoff=max(proof['stamp_ns'],head_latest[1],arrival.adapter.arrival)
                arrival.adapter=SearchVisionAdapter(scan,arrival.view_id,cutoff)
                scan.view_start=now;head_open=True;evidence.arrived(cutoff)
                emit(dict(scan.status(),observer_reason='head_ready_for_observation',head_stamp_ns=cutoff))
        try:
            row = json.loads(msg.data)
            if not isinstance(row,dict): raise ValueError('not an object')
            now=time.monotonic()
            result=arrival.adapter.consume(row,now,sensor_now_ns=node.get_clock().now().nanoseconds)
            evidence.observe(row,result,now)
            emit(result)
        except (ValueError,TypeError):
            invalidate('malformed_vision_message')

    subs.append(node.create_subscription(GoalStatusArray,a.action_status_topic,status_cb,
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    def tf_cb(msg):
        if not bound:
            return
        for item in msg.transforms:
            if (item.header.frame_id,item.child_frame_id) != ('odom','base_footprint'):
                continue
            ns = item.header.stamp.sec*1_000_000_000+item.header.stamp.nanosec
            try:
                # Explicitly buffer this message: listener callback order is not guaranteed.
                for transform in msg.transforms:
                    buffer.set_transform(transform,'handyman_readonly_tf')
                odom_pose = planar_pose(item.transform)
                map_tf = buffer.lookup_transform('map','base_footprint',Time(nanoseconds=ns))
                map_pose = planar_pose(map_tf.transform)
                emit(tf_observer.sample(odom_pose,map_pose,ns,
                    node.get_clock().now().nanoseconds,time.monotonic()))
            except (TransformException,ValueError):
                emit(tf_observer.invalidate('exact_stamp_tf_unavailable_or_invalid'))

    def binding_cb(msg):
        nonlocal bound, status_gate, arrival, tf_observer, scan, waiting_retry, last_pose
        if (scan.phase != 'navigate' and not waiting_retry):
            return
        try:
            import yaml
            binding=validate_binding(yaml.safe_load(msg.detail),a,node.get_clock().now().nanoseconds)
            if msg.message!='search_goal_accepted':
                raise ValueError('not_acceptance')
            if bound:
                if status_gate.done:
                    raise ValueError('current_goal_already_terminal')
                retry_bindings.defer(binding,msg,time.monotonic())
                emit(dict(scan.status(),observer_reason='retry_binding_deferred',goal_id=binding['goal_id']))
                return
            remaining=start+a.seconds-time.monotonic()
            if remaining<=0:
                raise ValueError('search_deadline_expired')
            retry_bindings.accept(binding)
            # New objects discard prior view stamps, counters, positions and TF window.
            scan=SearchScan(a.task_id,a.target,[dict(x=a.x,y=a.y,yaw=a.yaw)],time.monotonic(),
                timeout_s=remaining,view_s=min(a.view_seconds or remaining,remaining))
            evidence.reset()
            scan.evidence_ready=lambda now:evidence.snapshot(now)['view_data_usable']
            if room_gate is not None:scan.room_check=room_check
            status_gate=GoalStatusGate(binding['goal_id'])
            status_gate.active=True  # Actual action acceptance supplied by executor callback.
            arrival=SearchArrival(scan,status_gate.goal_id)
            tf_observer=TfSearchObserver(arrival,time.monotonic(), recover_transient_tf=a.recover_transient_tf, max_gap_s=.5)
            bound=True
            waiting_retry=False
            last_pose=None
            emit(dict(scan.status(),observer_reason='search_goal_bound',goal_id=status_gate.goal_id,
                      attempt=len(retry_bindings.seen),executor_generation=binding['executor_generation']))
            batches=list(status_cache)
            status_cache.clear()
            for batch in batches:
                status_cb(batch)
        except (ValueError,TypeError,AttributeError,yaml.YAMLError) as exc:
            emit(dict(scan.status(),observer_reason='binding_rejected',detail=str(exc)))

    def cancel_cb(msg):
        nonlocal bound,waiting_retry,last_pose
        import yaml
        try:
            row=yaml.safe_load(msg.detail)
        except yaml.YAMLError:
            return
        if not matches_task_cancel(msg.message,row,a.task_id):return
        bound=False;waiting_retry=False;last_pose=None;status_cache.clear()
        emit(cancel_observation(scan,arrival,tf_observer,retry_bindings))

    if a.binding_topic:
        from handyman_msgs.msg import HandymanMsg
        subs.append(node.create_subscription(HandymanMsg,a.binding_topic,binding_cb,10))
    if a.cancel_topic:
        from handyman_msgs.msg import HandymanMsg
        subs.append(node.create_subscription(HandymanMsg,a.cancel_topic,cancel_cb,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    if a.motion_source == 'tf':
        subs.append(node.create_subscription(TFMessage,'/tf',tf_cb,qos_profile_sensor_data))
    else:
        subs.append(node.create_subscription(Odometry,a.odom_topic,odom_cb,qos_profile_sensor_data))
    subs.append(node.create_subscription(String,a.vision_topic,vision_cb,10))
    try:
        emit(dict(scan.status(),observer_reason='ready_requires_new_active_goal',goal_id=status_gate.goal_id))
        while rclpy.ok() and time.monotonic()-start < a.seconds and (scan.phase in ('navigate','observe') or waiting_retry):
            rclpy.spin_once(node,timeout_sec=.05)
            scan.tick(time.monotonic())
            if a.motion_source == 'tf' and bound:
                failure = tf_observer.tick(time.monotonic())
                if failure is not None:
                    emit(failure)
            elif last_pose is not None and time.monotonic()-last_pose[0] > .5:
                invalidate('odometry_stream_timeout')
                last_pose = None
        emit(scan.status(),terminal=True)
    except (KeyboardInterrupt,ExternalShutdownException):
        scan.cancel(); emit(scan.status(),terminal=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        stream.close()


if __name__ == '__main__':
    main()

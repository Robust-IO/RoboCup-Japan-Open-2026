"""Read-only robot readiness audit. No command publishers or lifecycle writes."""
import argparse
import json
import math
from pathlib import Path
import time
from fresh_feedback_wait import wait_for_fresh


def validate_sample_times(joint_ns, map_ns, now_ns,*,joint_max_s=.4):
    if joint_max_s not in (.4,.6):raise ValueError('invalid_joint_age_limit')
    ages = dict(joints=(now_ns-joint_ns)/1e9, map_tf=(now_ns-map_ns)/1e9)
    for key, reason in (('joints','stale_joints'), ('map_tf','stale_base_tf')):
        if not math.isfinite(ages[key]) or not -.02 <= ages[key] <= (joint_max_s if key=='joints' else .4):
            raise RuntimeError(reason)
    return ages


def validate_feedback(joints, tolerances, publishers, subscribers, controller_state):
    if len(joints)<6 or any(not math.isfinite(v) for v in joints.values()):
        raise ValueError('finite_current_robot_state_required')
    opening=joints.get('hand_motor_joint',float('nan'))
    if len(tolerances) != 3 or any(not math.isfinite(x) or abs(x-y) > 1e-8 for x, y in zip(tolerances, [.06, .08, .06])):
        raise ValueError('navigation_parameters_differ_no_changes_made')
    if any(publishers.values()) or any(v != 1 for v in subscribers.values()):
        raise ValueError('robot_controller_not_exclusive')
    if controller_state not in (1, 2):
        raise ValueError('navigation_must_be_idle_unconfigured_or_inactive')
    return dict(ready=True, initial_hand_open=opening>=.60, holding_object=None, initial_pose_is_acceptance_gate=False,
                navigation_parameters_changed=False, robot_commands_sent=False)


def main():
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer, TransformListener
    from rcl_interfaces.srv import GetParameters
    from lifecycle_msgs.srv import GetState
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--efficient-transfer', action='store_true')
    parser.add_argument('--joint-feedback-max-age-s',type=float,choices=(.4,.6),default=.4)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('output_exists')
    rclpy.init()
    node = rclpy.create_node('automatic_transfer_readonly_preflight')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    data = {}
    sub = node.create_subscription(JointState, '/hsrb/joint_states', lambda m: data.update(joints=m), qos_profile_sensor_data)
    report = dict(ready=False, robot_commands_sent=False)
    def call(client, request):
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('readiness_service_missing')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError('readiness_read_timeout')
        return future.result()
    def current_feedback():
        # Drain work already received instead of accepting a queued old sample.
        for _ in range(20):rclpy.spin_once(node, timeout_sec=0)
        if 'joints' not in data:raise RuntimeError('stale_joints')
        joint = data['joints']
        if not buffer.can_transform('map', 'base_footprint', Time()):
            raise RuntimeError('stale_base_tf')
        tf = buffer.lookup_transform('map', 'base_footprint', Time())
        now = node.get_clock().now().nanoseconds
        joint_ns, map_ns = Time.from_msg(joint.header.stamp).nanoseconds, Time.from_msg(tf.header.stamp).nanoseconds
        report['feedback_ages_s'] = dict(joints=(now-joint_ns)/1e9, map_tf=(now-map_ns)/1e9)
        validate_sample_times(joint_ns, map_ns, now,joint_max_s=args.joint_feedback_max_age_s)
        report.update(joint_stamp_ns=joint_ns, map_stamp_ns=map_ns, feedback_checked_ns=now)
        return joint, tf
    def fresh_feedback():
        def waiting():report['feedback_waits'] = report.get('feedback_waits',0)+1
        from gate_recovery import transient_observation_error,RecoveryStatus
        recovery=RecoveryStatus(args.output.parent)
        while True:
            try:
                result=wait_for_fresh(current_feedback,lambda dt:rclpy.spin_once(node,timeout_sec=dt),waiting)
                recovery.recovered();return result
            except Exception as exc:
                if not transient_observation_error(exc):raise
                recovery.waiting(str(exc));rclpy.spin_once(node,timeout_sec=.05)
    try:
        # Wait for first live feedback without a startup wall-clock cutoff.
        while 'joints' not in data or not buffer.can_transform('map','base_footprint',Time()):
            rclpy.spin_once(node,timeout_sec=.05)
        from navigation_entry import repair_navigation,precision_matches
        from gate_recovery import RecoveryStatus
        recovery=RecoveryStatus(args.output.parent)
        while True:
            try:
                repair_navigation(node,root=args.output.parent)
                getter=node.create_client(GetParameters,'/controller_server/get_parameters')
                try:
                    request=GetParameters.Request()
                    request.names=['general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','FollowPath.xy_goal_tolerance']
                    tolerances=[v.double_value if v.type==3 else None for v in call(getter,request).values]
                finally:node.destroy_client(getter)
                state_client=node.create_client(GetState,'/controller_server/get_state')
                try:state=call(state_client,GetState.Request()).current_state.id
                finally:node.destroy_client(state_client)
                if not precision_matches(tolerances) or state not in (1,2):
                    recovery.waiting('navigation_readiness_changed_repair_again');continue
                recovery.recovered();break
            except (RuntimeError,TimeoutError) as exc:
                recovery.waiting(str(exc));rclpy.spin_once(node,timeout_sec=.1)
        # Parameter/lifecycle reads also consume time. Acquire the feedback that
        # will be checked and reported AFTER these calls, never before them.
        joint, tf = fresh_feedback()
        t, q = tf.transform.translation, tf.transform.rotation
        pose = [t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
        topics = ['/hsrb/' + tail for tail in ('command_velocity', 'omni_base_controller/command', 'arm_trajectory_controller/command', 'gripper_controller/command', 'head_trajectory_controller/command')]
        publishers = {t: node.count_publishers(t) for t in topics}
        subscribers = {t: node.count_subscribers(t) for t in topics[1:]}
        joints = dict(zip(joint.name, joint.position))
        report.update(validate_feedback(joints, tolerances, publishers, subscribers, state),
                      joints=joints, base_map_pose=pose, navigation_tolerances=tolerances,
                      command_publishers=publishers, command_subscribers=subscribers, controller_state=state)
    except Exception as exc:
        report['reason'] = str(exc)
    finally:
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report))
    if not report['ready']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

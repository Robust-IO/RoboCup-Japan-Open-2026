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
    expected = dict(arm_lift_joint=0., arm_flex_joint=0., arm_roll_joint=0.,
                    wrist_flex_joint=-math.pi/2, wrist_roll_joint=0.)
    if any(k not in joints or not math.isfinite(joints[k]) or abs(joints[k]-v) > .005 for k, v in expected.items()):
        raise ValueError('empty_tucked_start_required')
    opening = joints.get('hand_motor_joint', float('nan'))
    if not math.isfinite(opening) or min(abs(opening), abs(opening-.65)) > .005:
        raise ValueError('hand_may_be_holding_object')
    if len(tolerances) != 3 or any(not math.isfinite(x) or abs(x-y) > 1e-8 for x, y in zip(tolerances, [.06, .08, .06])):
        raise ValueError('navigation_parameters_differ_no_changes_made')
    if any(publishers.values()) or any(v != 1 for v in subscribers.values()):
        raise ValueError('robot_controller_not_exclusive')
    if controller_state not in (1, 2):
        raise ValueError('navigation_must_be_idle_unconfigured_or_inactive')
    return dict(ready=True, initial_hand_open=abs(opening-.65) < .005, holding_object=False,
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
        return wait_for_fresh(current_feedback, lambda dt:rclpy.spin_once(node,timeout_sec=dt), waiting)
    try:
        if args.efficient_transfer:
            from transfer_efficiency import wait_ready
            wait_ready(lambda: 'joints' in data and buffer.can_transform('map','base_footprint',Time()),
                       lambda dt:rclpy.spin_once(node,timeout_sec=dt))
        else:
            end = time.monotonic() + 4
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.03)
                for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
        getter = node.create_client(GetParameters, '/controller_server/get_parameters')
        request = GetParameters.Request()
        request.names = ['general_goal_checker.xy_goal_tolerance', 'general_goal_checker.yaw_goal_tolerance', 'FollowPath.xy_goal_tolerance']
        tolerances = [v.double_value for v in call(getter, request).values]
        state = call(node.create_client(GetState, '/controller_server/get_state'), GetState.Request()).current_state.id
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

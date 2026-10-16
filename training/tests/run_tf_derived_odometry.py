"""Isolated ROS 2 integration test; never run against the live Unity domain."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '73' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires isolated domain 73 and ROS_LOCALHOST_ONLY=1')
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = rclpy.create_node('synthetic_odometry_test')
    publisher = node.create_publisher(TFMessage, '/tf', 10)
    command_publisher = node.create_publisher(Twist, '/hsrb/command_velocity', 10)
    other = None
    received = []
    subscription = node.create_subscription(Odometry, '/handyman/odometry/derived', received.append, 10)
    output = Path(tempfile.mkdtemp(prefix='derived-odometry-')) / 'samples.jsonl'
    script = Path(__file__).resolve().parents[1] / 'scripts/probe_derived_odometry.py'
    child = subprocess.Popen([sys.executable, str(script), '--run', '--publish', '--observe-commands', '--seconds', '9', '--output', str(output)])
    start = time.monotonic()
    next_send = start
    try:
        while child.poll() is None and time.monotonic() - start < 13:
            now = time.monotonic()
            elapsed = now - start
            if elapsed > 6.5 and other is None:
                other = node.create_publisher(TFMessage, '/tf', 10)
            # Continuous forward motion, pause in reception, then second authority.
            if now >= next_send and not 3.5 < elapsed < 4.2:
                tf = TransformStamped()
                tf.header.stamp = node.get_clock().now().to_msg()
                tf.header.frame_id = 'odom'
                tf.child_frame_id = 'base_footprint'
                tf.transform.translation.x = .1 * elapsed
                tf.transform.rotation.w = 1.
                (other if elapsed > 6.5 else publisher).publish(TFMessage(transforms=[tf]))
                command=Twist();command.linear.x=.1
                command_publisher.publish(command)
                next_send = now + .1
            rclpy.spin_once(node, timeout_sec=.01)
        assert child.wait(timeout=2) == 0
        rows = [json.loads(line) for line in output.read_text().splitlines()]
        commands=[row for row in rows if row.get('record_type')=='observed_command']
        assert len(commands)>=10
        assert all(row['finite'] and row['command']['vx']==.1 for row in commands)
        rows=[row for row in rows if row.get('record_type')=='estimate']
        valid = [row for row in rows if row['valid']]
        assert len(valid) >= 10 and len(received) >= 10, (len(valid), len(received))
        assert all(abs(row['twist']['vx']-.1) < .015 for row in valid)
        assert all(message.header.frame_id == 'odom' and message.child_frame_id == 'base_footprint' for message in received)
        assert all(abs(message.twist.twist.linear.x-.1) < .015 for message in received)
        reasons = {row['reason'] for row in rows}
        assert reasons & {'receive_timeout_or_clock_reset', 'sensor_clock_stale_or_reset'}, reasons
        conflict = next(i for i, row in enumerate(rows) if row['reason'] == 'multiple_or_unknown_tf_publishers')
        assert not any(row['valid'] for row in rows[conflict:])
        assert all('twist' not in row for row in rows if not row['valid'])
        print(json.dumps(dict(passed=True, valid=len(valid), received=len(received), observed_commands=len(commands), output=str(output))))
    finally:
        if child.poll() is None:
            child.terminate()
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait(timeout=3)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

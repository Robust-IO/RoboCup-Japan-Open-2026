#!/usr/bin/env python3
"""Exercise production TF lookup with a real tf2 buffer, without a simulator."""
import math
import sys
from pathlib import Path

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/cleanup_vision_ros2/scripts'))
from cleanup_detection_node import CleanupDetectionNode


def main():
    rclpy.init()
    logger_node = Node('tf_boundary_test')
    try:
        cases = [
            # Logged small future gaps, static camera.
            (0.011, 0., 0., True), (0.029, 0., 0., True),
            # Disconnect/stale transform, moving camera, and an older image.
            (0.258, 0., 0., False), (0.020, 0.02, 0., False),
            (0.020, 0., 0.10, False), (-0.010, 0., 0., False),
        ]
        for gap, movement, yaw, expected in cases:
            detector = CleanupDetectionNode.__new__(CleanupDetectionNode)
            detector.tf_buffer = tf2_ros.Buffer()
            detector.get_logger = logger_node.get_logger
            for seconds, x, angle in ((10., 0., 0.), (10.1, movement, yaw)):
                transform = TransformStamped()
                transform.header.frame_id = 'odom'
                transform.child_frame_id = 'camera'
                transform.header.stamp = rclpy.time.Time(seconds=seconds).to_msg()
                transform.transform.translation.x = x
                transform.transform.rotation.z = math.sin(angle / 2)
                transform.transform.rotation.w = math.cos(angle / 2)
                detector.tf_buffer.set_transform(transform, 'test')
            requested = rclpy.time.Time(seconds=10.1 + gap)
            result = detector._stationary_tf_boundary('odom', 'camera', requested)
            assert (result is not None) == expected, (gap, movement, yaw)
            if expected:
                actual = detector._lookup_transform('odom', 'camera', requested)
                assert actual is not None
                assert actual.header.stamp == transform.header.stamp
        print(f'{len(cases)} real-buffer TF boundary cases passed')
    finally:
        logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

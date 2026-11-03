#!/usr/bin/env python3
"""Exercise the actual detector's depth/TF methods using a known RGB-D scene.

Run after sourcing ROS and install/setup.bash with the project's Python.
This does not load YOLO/MediaPipe models or command the robot.
"""
import math
import sys
from pathlib import Path

import numpy as np
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, quaternion_matrix

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/cleanup_vision_ros2/scripts'))
from cleanup_detection_node import CleanupDetectionNode


def verify():
    cases = 0
    for scale in (1.0, 0.001):
        for yaw, pitch in ((0, 0), (0.6, -0.4), (-0.9, 0.3)):
            # An isolated foreground arm over a wall. Depth is synthesized by
            # projecting a known 3-D line, with a deliberately corrupted patch.
            node = CleanupDetectionNode.__new__(CleanupDetectionNode)
            node.fx = node.fy = 554.0
            node.cx_cam, node.cy_cam = 320.0, 240.0
            node.depth_frame_usable = True
            node.depth_scale = scale
            node.depth_image = np.full((480, 640), 4.0/scale)
            u0, v0, u1, v1 = 180.0, 190.0, 310.0, 225.0
            z0, z1 = 1.8, 1.5
            for u in range(178, 313):
                t = (u-u0)/(u1-u0)
                v = int(round(v0+t*(v1-v0)))
                z = 1.0/((1-t)/z0+t/z1)
                node.depth_image[v-1:v+2, u] = z/scale
            # Drop foreground near one sampled location to expose background.
            node.depth_image[197:211, 224:234] = 4.0/scale
            fit = node._fit_arm_depth_direction(u0, v0, u1, v1)
            assert fit is not None, (scale, yaw, pitch)
            origin, direction, error, support, length = fit
            expected_start = np.array([(u0-320)*z0/554, (v0-240)*z0/554, z0])
            expected_end = np.array([(u1-320)*z1/554, (v1-240)*z1/554, z1])
            assert np.linalg.norm(np.array(origin)-expected_start) < 0.02
            expected_direction = expected_end-expected_start
            expected_direction /= np.linalg.norm(expected_direction)
            assert np.dot(direction, expected_direction) > math.cos(math.radians(3))
            assert support >= 7 and error < 0.02

            transform = TransformStamped()
            q = quaternion_from_euler(0, pitch, yaw)
            transform.transform.rotation.x = float(q[0])
            transform.transform.rotation.y = float(q[1])
            transform.transform.rotation.z = float(q[2])
            transform.transform.rotation.w = float(q[3])
            transform.transform.translation.x = 1.1
            transform.transform.translation.y = -0.7
            transform.transform.translation.z = 1.2
            node.frame_camera_transform = transform
            node.frame_camera_transform_resolved = True
            rotation = quaternion_matrix(q)[:3, :3]
            actual = node._cam3d_to_odom(*expected_start)
            expected = rotation @ expected_start + np.array([1.1, -0.7, 1.2])
            assert np.linalg.norm(np.array(actual)-expected) < 1e-9
            fallback = node._mediapipe_world_vector_to_odom(expected_direction)
            assert np.dot(fallback, rotation @ expected_direction) > 0.999999
            cases += 1
    print(f'{cases} detector-level synthetic RGB-D/TF cases passed')


if __name__ == '__main__':
    verify()

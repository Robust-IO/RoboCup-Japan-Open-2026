#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

import perception_visualization  # noqa: E402


class PerceptionVisualizationTest(unittest.TestCase):
    def test_build_head_marker_array_contains_avatar_pointing_and_object_markers(self):
        avatar = SimpleNamespace(
            header=Header(frame_id='odom'),
            is_valid=True,
            origin_odom=Point(x=1.0, y=2.0, z=1.3),
            confidence=0.92,
            body_yaw=0.35,
        )

        obj = SimpleNamespace(
            class_name='soysauce',
            confidence=0.88,
            has_3d_position=True,
            position=Point(x=2.5, y=-0.2, z=0.8),
        )
        objects = SimpleNamespace(
            header=Header(frame_id='odom'),
            objects=[obj],
        )

        pointing = SimpleNamespace(
            header=Header(frame_id='odom'),
            is_valid=True,
            confidence=0.81,
            origin=Point(x=1.0, y=2.0, z=1.3),
            direction=Vector3(x=0.0, y=1.0, z=0.0),
        )

        markers = perception_visualization.build_head_marker_array(
            avatar,
            objects,
            pointing,
        )

        self.assertEqual('odom', markers.markers[0].header.frame_id)
        self.assertEqual(Marker.DELETEALL, markers.markers[0].action)
        namespaces = {marker.ns for marker in markers.markers[1:]}
        self.assertIn('avatar', namespaces)
        self.assertIn('pointing', namespaces)
        self.assertIn('objects', namespaces)

    def test_build_hand_marker_array_contains_command_arrow_and_status_text(self):
        alignment = SimpleNamespace(
            header=Header(frame_id='hand_camera_frame'),
            is_target_found=True,
            target_class='soysauce',
            pixel_error_x=-0.12,
            pixel_error_y=0.08,
            recommended_linear_x=0.04,
            recommended_linear_y=-0.02,
            recommended_lift_delta=0.01,
            in_grasp_window=False,
            confidence=0.76,
        )

        markers = perception_visualization.build_hand_marker_array(alignment)

        self.assertEqual('hand_camera_frame', markers.markers[0].header.frame_id)
        self.assertEqual(Marker.DELETEALL, markers.markers[0].action)
        namespaces = {marker.ns for marker in markers.markers[1:]}
        self.assertIn('hand_alignment', namespaces)
        self.assertIn('hand_status', namespaces)


if __name__ == '__main__':
    unittest.main()

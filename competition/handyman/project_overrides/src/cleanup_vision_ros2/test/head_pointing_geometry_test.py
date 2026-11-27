#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

import pointing_utils  # noqa: E402


class HeadPointingGeometryTest(unittest.TestCase):
    def test_expand_person_crop_clamps_to_image_bounds(self):
        rect = pointing_utils.expand_person_crop(
            cx=30.0,
            cy=40.0,
            width=80.0,
            height=120.0,
            image_width=640,
            image_height=480,
            width_scale=1.5,
            height_scale=1.6,
        )

        self.assertEqual((0, 0, 90, 136), rect)

    def test_project_pointing_pixel_prefers_arm_direction(self):
        point_u, point_v, straightness = pointing_utils.project_pointing_pixel(
            shoulder=(120.0, 160.0),
            elbow=(170.0, 150.0),
            wrist=(230.0, 140.0),
            index_tip=(260.0, 132.0),
        )

        self.assertGreater(point_u, 260.0)
        self.assertLess(point_v, 140.0)
        self.assertGreater(straightness, 0.90)

    def test_smooth_yaw_rejects_large_low_confidence_jumps(self):
        accepted, smoothed = pointing_utils.smooth_pointing_yaw(
            previous_yaw=0.10,
            raw_yaw=1.40,
            confidence=0.25,
            alpha=0.35,
            max_low_conf_jump=0.55,
        )

        self.assertFalse(accepted)
        self.assertAlmostEqual(0.10, smoothed, places=6)

    def test_select_pointing_endpoint_prefers_depth_when_near_wrist_depth(self):
        endpoint, mode = pointing_utils.select_pointing_endpoint(
            origin_cam=(0.18, -0.02, 2.00),
            point_cam=(0.48, -0.08, 2.07),
            point_ray=pointing_utils.pixel_to_camera_ray(
                u=430.0, v=220.0, fx=554.0, fy=554.0, cx=320.0, cy=240.0),
        )

        self.assertEqual('depth', mode)
        self.assertAlmostEqual(2.07, endpoint[2], places=6)

    def test_select_pointing_endpoint_falls_back_to_wrist_depth_projection(self):
        endpoint, mode = pointing_utils.select_pointing_endpoint(
            origin_cam=(0.18, -0.02, 2.00),
            point_cam=(1.10, -0.30, 3.40),
            point_ray=pointing_utils.pixel_to_camera_ray(
                u=430.0, v=220.0, fx=554.0, fy=554.0, cx=320.0, cy=240.0),
        )

        self.assertEqual('wrist_depth', mode)
        self.assertAlmostEqual(2.00, endpoint[2], places=6)
        self.assertGreater(endpoint[0], 0.18)


if __name__ == '__main__':
    unittest.main()

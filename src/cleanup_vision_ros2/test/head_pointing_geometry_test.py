#!/usr/bin/env python3

import sys
import math
import unittest
import ast
from types import SimpleNamespace
from unittest.mock import Mock
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))
DETECTION_SOURCE = (SCRIPTS_DIR / 'cleanup_detection_node.py').read_text()

import pointing_utils  # noqa: E402


class HeadPointingGeometryTest(unittest.TestCase):
    def test_image_pyramid_merges_duplicates_without_merging_instances(self):
        boxes = [[0,0,100,100,.6,1], [1,1,101,101,.9,1],
                 [0,0,100,100,.8,2], [200,0,300,100,.7,1]]
        self.assertEqual(pointing_utils.merge_multiscale_boxes(boxes), [1,2,3])
        self.assertEqual(pointing_utils.merge_multiscale_boxes([]), [])

    def test_finger_confirmed_hand_recovers_same_frame_forearm(self):
        import numpy as np
        cls = next(n for n in ast.parse(DETECTION_SOURCE).body
                   if isinstance(n, ast.ClassDef) and n.name == 'CleanupDetectionNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_associate_pointing_hand')
        scope = dict(np=np, math=math, _landmark_visibility=lambda p:p.visibility,
                     _L_ELBOW=13, _L_WRIST=15, _L_INDEX=19,
                     _R_ELBOW=14, _R_WRIST=16, _R_INDEX=20)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<association>', 'exec'), scope)
        estimate = scope['_associate_pointing_hand']
        lm=[SimpleNamespace(x=.5,y=.5,z=0,visibility=0) for _ in range(33)]
        lm[13]=SimpleNamespace(x=.4,y=.4,z=0,visibility=.9)
        lm[15]=SimpleNamespace(x=.4,y=.6,z=0,visibility=.9)
        results=SimpleNamespace(pose_landmarks=[lm],pose_world_landmarks=[])
        hand=dict(hand_wrist=(256,288),pu=260,pv=310,conf=.7)
        p=estimate(None,hand,results,640,480)
        self.assertEqual((p['wu'],p['wv']),(256,192))
        self.assertEqual((p['pu'],p['pv']),(260,310))
        self.assertTrue(p['finger_refined'])
        # Never attach an unrelated/distant hand or an invisible elbow.
        hand['hand_wrist']=(600,10)
        self.assertIsNone(estimate(None,hand,results,640,480))
        hand['hand_wrist']=(float('nan'),288)
        self.assertIsNone(estimate(None,hand,results,640,480))
        hand['hand_wrist']=(256,288); lm[13].visibility=0
        self.assertIsNone(estimate(None,hand,results,640,480))

    def test_historical_full_arm_uses_endpoints_then_pose_with_real_origin(self):
        import numpy as np
        tree = ast.parse(DETECTION_SOURCE)
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == 'CleanupDetectionNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_historical_arm_ray')
        scope = {'np': np}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<arm ray>', 'exec'), scope)
        estimate = scope['_historical_arm_ray']
        detector = SimpleNamespace(
            _foreground_pixel_to_cam3d=Mock(side_effect=[(0,0,2,0),(.2,0,1.8,0)]),
            _cam3d_to_odom=Mock(return_value=(3.,1.,1.)),
            _cam_vector_to_odom=Mock(return_value=(1.,0.,-1.)))
        best = dict(wu=100,wv=200,pu=150,pv=250,joint_refined=True)
        origin, direction = estimate(detector,best,(0,1,0),'capture')
        self.assertEqual(origin,(3.,1.,1.))
        np.testing.assert_allclose(direction,(1/np.sqrt(2),0,-1/np.sqrt(2)))
        self.assertEqual(detector._cam3d_to_odom.call_args.args[-1],'capture')
        self.assertEqual(detector._cam_vector_to_odom.call_args.args[-1],'capture')
        detector._foreground_pixel_to_cam3d.side_effect=[(0,0,2,0),None]
        self.assertEqual(estimate(detector,best,(0,1,0),'capture')[1],(0,1,0))
        detector._foreground_pixel_to_cam3d.side_effect=[None]
        self.assertIsNone(estimate(detector,best,(0,1,0),'capture'))
        detector._foreground_pixel_to_cam3d.side_effect=[(0,0,2,0),None]
        self.assertIsNone(estimate(detector,best,(float('nan'),1,0),'capture'))

    def test_tracking_prefers_visible_shoulders_over_lower_hips(self):
        lm = [[0.5, 0.5, 0.0, 0.0] for _ in range(33)]
        lm[11] = [0.4, 0.3, 0.0, 0.9]
        lm[12] = [0.6, 0.3, 0.0, 0.9]
        lm[23] = [0.4, 0.7, 0.0, 0.9]
        lm[24] = [0.6, 0.7, 0.0, 0.9]
        self.assertEqual(pointing_utils.shoulder_tracking_center(lm, 640, 480),
                         (320.0, 144.0))
        lm[11][3] = 0.0
        self.assertEqual(pointing_utils.shoulder_tracking_center(lm, 640, 480),
                         (384.0, 144.0))
        lm[12][3] = 0.0
        self.assertEqual(pointing_utils.shoulder_tracking_center(lm, 640, 480),
                         (320.0, 336.0))
        lm[23][0] = float('nan'); lm[24][3] = 0.0
        self.assertIsNone(pointing_utils.shoulder_tracking_center(lm, 640, 480))

    def test_wrist_crop_maps_finger_back_to_capture_pixels(self):
        import numpy as np
        import cv2
        tree = ast.parse(DETECTION_SOURCE)
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == 'CleanupDetectionNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_estimate_close_hand_pointing')
        hand = [SimpleNamespace(x=0.5, y=0.5, z=0.0) for _ in range(21)]
        hand[0].y=0.9; hand[5].y=0.7; hand[6].y=0.5; hand[8].y=0.2
        result = SimpleNamespace(hand_landmarks=[hand], handedness=[], hand_world_landmarks=[])
        detector = SimpleNamespace(
            gesture_audit={}, hand_detector=SimpleNamespace(detect=Mock(return_value=result)),
            _fit_arm_depth_direction=Mock(return_value=None),
            _foreground_pixel_to_cam3d=Mock(return_value=None))
        namespace = dict(np=np, cv2=cv2, math=math,
                         mp=SimpleNamespace(Image=lambda **kw: kw,
                                            ImageFormat=SimpleNamespace(SRGB=0)),
                         is_extended_index_finger=pointing_utils.is_extended_index_finger,
                         _INDEX_TIP=8, _INDEX_PIP=6, _INDEX_MCP=5, _HAND_WRIST=0)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<hand crop>', 'exec'), namespace)
        estimate = namespace['_estimate_close_hand_pointing']
        rgb=np.zeros((480,640,3),dtype=np.uint8)
        p=estimate(detector,rgb,None,(100,200,228,328))
        self.assertTrue(p['is_pointing'])
        self.assertAlmostEqual(p['wu'],164)
        self.assertAlmostEqual(p['wv'],264)
        self.assertAlmostEqual(p['pu'],164)
        self.assertAlmostEqual(p['pv'],225.6)
        self.assertFalse(p['depth_direction_valid'])
        detector._fit_arm_depth_direction.assert_called_once_with(164.0,289.6,164.0,225.6)
        # Left/right classification certainty cannot promote geometric quality.
        result.handedness = [[SimpleNamespace(score=0.999)]]
        classified = estimate(detector,rgb,None,(100,200,228,328))
        self.assertEqual(classified['conf'], p['conf'])
        self.assertEqual(classified['geometry_source'], 'hand')
        # A folded index stays rejected after enlargement.
        hand[8].y=0.75
        self.assertFalse(estimate(detector,rgb,None,(100,200,228,328))['is_pointing'])

    def test_detection_rate_limit_does_not_read_wall_clock(self):
        # Execute the production method without importing ROS/ML libraries.
        # No RGB-D pair is available, so only its scheduling path runs.
        tree = ast.parse(DETECTION_SOURCE)
        node = next(n for n in tree.body
                    if isinstance(n, ast.ClassDef) and n.name == 'CleanupDetectionNode')
        method = next(n for n in node.body
                      if isinstance(n, ast.FunctionDef) and n.name == 'run_detection')
        fake_time = SimpleNamespace(monotonic=Mock())
        namespace = {'time': fake_time}
        exec(compile(ast.Module(body=[method], type_ignores=[]),
                     '<production run_detection>', 'exec'), namespace)
        detector = SimpleNamespace(
            detection_enabled=True, last_detect_time=100.0,
            detect_interval_sec=0.2, _select_rgbd_pair=Mock(return_value=None),
            get_clock=Mock(side_effect=AssertionError('wall clock used for scheduling')))
        # Detection remains periodic even if wall time is unavailable or jumps.
        for tick in (100.1, 100.25, 100.3, 100.5, 100.75):
            fake_time.monotonic.return_value = tick
            namespace['run_detection'](detector)
        self.assertEqual(detector._select_rgbd_pair.call_count, 3)
        self.assertEqual(detector.last_detect_time, 100.75)
        detector.detection_enabled = False
        fake_time.monotonic.return_value = 101.0
        namespace['run_detection'](detector)
        self.assertEqual(detector._select_rgbd_pair.call_count, 3)

    def test_mediapipe_depth_is_not_reflected(self):
        direction = pointing_utils.mediapipe_to_optical((0.3, 0.2, -0.8))
        self.assertLess(direction[2], 0.0)
        self.assertAlmostEqual(1.0, sum(x*x for x in direction))

    def test_image_line_plane_contains_all_projected_points(self):
        normal = pointing_utils.pointing_plane_normal(
            (320, 240), (420, 290), 554, 554, 320, 240)
        self.assertIsNotNone(normal)
        for depth in (0.5, 1.0, 3.0, 5.0):
            for t in (0.0, 0.4, 1.0, 3.0):
                point = (100*t/554*depth, 50*t/554*depth, depth)
                self.assertAlmostEqual(0.0, sum(a*b for a, b in zip(normal, point)))

    def test_image_forward_halfspace_rejects_points_behind_the_hand(self):
        forward = pointing_utils.pointing_forward_normal(
            (320, 240), (420, 290), 554, 554, 320, 240)
        self.assertGreater(sum(a*b for a, b in zip(forward, (0.4, 0.2, 2))), 0)
        self.assertLess(sum(a*b for a, b in zip(forward, (-0.4, -0.2, 2))), 0)

    def test_foreground_depth_returns_supported_surface_not_percentile_blend(self):
        values = [0.3, 1.8, 1.801, 1.802] + [4.0]*20
        self.assertAlmostEqual(1.801, pointing_utils.robust_foreground_depth(values))
        self.assertIsNone(pointing_utils.robust_foreground_depth([float('nan'), 0, 1.0]))

    def test_projective_fit_recovers_known_arm_despite_background_pixels(self):
        start, end = (0.05, 0.1, 1.8), (0.4, 0.14, 1.35)
        samples = []
        for index in range(9):
            t = index/8
            inv_z = (1-t)/start[2] + t/end[2]
            z = 1/inv_z
            x = ((1-t)*start[0]/start[2] + t*end[0]/end[2])*z
            y = ((1-t)*start[1]/start[2] + t*end[1]/end[2])*z
            samples.append((t, x, y, z))
        samples[3] = (3/8, 1.0, 0.3, 4.0)
        fitted = pointing_utils.fit_projective_depth_line(samples)
        self.assertIsNotNone(fitted)
        origin, direction, error, count, length = fitted
        expected_length = math.dist(start, end)
        self.assertLess(math.dist(start, origin), 1e-7)
        self.assertAlmostEqual(expected_length, length)
        self.assertGreaterEqual(count, 8)
        self.assertLess(error, 1e-7)
        self.assertGreater(sum(direction[k]*(end[k]-start[k])/expected_length
                               for k in range(3)), 0.9999)

    def test_projective_fit_rejects_background_patch_and_impossible_limb(self):
        partial = [(0.3+i*0.04, i*0.04, 0.0, 3.0) for i in range(6)]
        self.assertIsNone(pointing_utils.fit_projective_depth_line(partial))
        oversized = [(i/8, i*0.2, 0.0, 3.0) for i in range(9)]
        self.assertIsNone(pointing_utils.fit_projective_depth_line(oversized))

    def test_close_range_extended_index_is_accepted(self):
        self.assertTrue(pointing_utils.is_extended_index_finger(
            wrist=(100.0, 120.0),
            index_mcp=(120.0, 105.0),
            index_pip=(140.0, 94.0),
            index_tip=(168.0, 82.0),
        ))

    def test_close_range_curled_index_is_rejected(self):
        self.assertFalse(pointing_utils.is_extended_index_finger(
            wrist=(100.0, 120.0),
            index_mcp=(120.0, 105.0),
            index_pip=(140.0, 94.0),
            index_tip=(132.0, 108.0),
        ))

    def test_pose_loss_uses_full_frame_close_hand_fallback(self):
        self.assertIn(
            'def _estimate_close_hand_pointing(', DETECTION_SOURCE)
        self.assertIn(
            'if not results.pose_landmarks:\n'
            '            return self._estimate_close_hand_pointing(',
            DETECTION_SOURCE)
        self.assertIn("else 'close-range hand fallback'", DETECTION_SOURCE)

    def test_rgb_and_depth_are_paired_from_timestamped_queues(self):
        self.assertIn('self.rgb_frames = deque(maxlen=10)', DETECTION_SOURCE)
        self.assertIn('self.depth_frames = deque(maxlen=10)', DETECTION_SOURCE)
        self.assertIn('def _select_rgbd_pair(self):', DETECTION_SOURCE)
        self.assertIn('select_synchronized_pair(', DETECTION_SOURCE)
        self.assertIn('time.monotonic() - newest_rgb[3] < 0.8',
                      DETECTION_SOURCE)

    def test_timestamp_pairing_uses_newest_synchronized_rgb(self):
        pair = pointing_utils.select_synchronized_pair(
            rgb_timestamps=[1_000_000_000, 2_000_000_000],
            depth_timestamps=[990_000_000, 2_400_000_000, 2_030_000_000],
        )
        self.assertEqual((1, 2, 30_000_000), pair)

    def test_timestamp_pairing_rejects_only_stale_depth(self):
        pair = pointing_utils.select_synchronized_pair(
            rgb_timestamps=[2_000_000_000],
            depth_timestamps=[1_500_000_000],
        )
        self.assertIsNone(pair)

    def test_multi_point_depth_fit_rejects_background_outlier(self):
        samples = [
            (0.0, 0.00, 0.00, 2.00),
            (0.2, 0.10, 0.00, 1.90),
            (0.4, 0.20, 0.00, 1.80),
            (0.5, 1.80, 0.50, 4.50),  # background contamination
            (0.6, 0.30, 0.00, 1.70),
            (0.8, 0.40, 0.00, 1.60),
            (1.0, 0.50, 0.00, 1.50),
        ]
        fitted = pointing_utils.fit_parametric_line_3d(samples)
        self.assertIsNotNone(fitted)
        _, direction, rmse, count, _ = fitted
        self.assertGreaterEqual(count, 6)
        self.assertLess(rmse, 0.02)
        self.assertGreater(direction[0], 0.65)
        self.assertLess(direction[2], -0.65)

    def test_multi_point_depth_fit_rejects_incoherent_depth(self):
        samples = [
            (0.0, 0.0, 0.0, 1.0),
            (0.2, 0.8, -0.4, 3.2),
            (0.4, -0.5, 0.7, 1.3),
            (0.6, 1.4, 0.2, 4.0),
            (0.8, -0.7, -0.5, 2.1),
            (1.0, 0.3, 0.9, 5.0),
        ]
        self.assertIsNone(pointing_utils.fit_parametric_line_3d(samples))

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

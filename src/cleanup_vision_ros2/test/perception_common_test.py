#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path
from unittest import mock


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

import perception_common  # noqa: E402


class PerceptionCommonTest(unittest.TestCase):
    def test_select_pose_backend_prefers_tasks_when_asset_exists(self):
        backend = perception_common.select_pose_backend(
            has_mediapipe_tasks=True,
            pose_model_path='/tmp/pose_landmarker.task',
            has_mediapipe_solutions=True,
        )
        self.assertEqual('tasks', backend)

    def test_select_pose_backend_falls_back_to_solutions(self):
        backend = perception_common.select_pose_backend(
            has_mediapipe_tasks=True,
            pose_model_path=None,
            has_mediapipe_solutions=True,
        )
        self.assertEqual('solutions', backend)

    def test_select_pose_backend_disables_when_pose_is_unavailable(self):
        backend = perception_common.select_pose_backend(
            has_mediapipe_tasks=False,
            pose_model_path=None,
            has_mediapipe_solutions=False,
        )
        self.assertEqual('disabled', backend)

    def test_describe_pose_backend_unavailable_reports_missing_task_asset(self):
        reason = perception_common.describe_pose_backend_unavailable(
            has_mediapipe=True,
            has_mediapipe_tasks=True,
            pose_model_path=None,
            has_mediapipe_solutions=False,
        )
        self.assertIn('PoseLandmarker model asset not found', reason)
        self.assertIn('fallback unavailable', reason)

    def test_describe_pose_backend_unavailable_reports_missing_mediapipe(self):
        reason = perception_common.describe_pose_backend_unavailable(
            has_mediapipe=False,
            has_mediapipe_tasks=False,
            pose_model_path=None,
            has_mediapipe_solutions=False,
        )
        self.assertEqual('MediaPipe unavailable', reason)

    def test_load_yolo_class_returns_none_with_clear_error_on_import_failure(self):
        with mock.patch.object(
            perception_common.importlib,
            'import_module',
            side_effect=ImportError('missing ultralytics'),
        ):
            yolo_class, error = perception_common.load_yolo_class()

        self.assertIsNone(yolo_class)
        self.assertIn('ultralytics', error)


if __name__ == '__main__':
    unittest.main()

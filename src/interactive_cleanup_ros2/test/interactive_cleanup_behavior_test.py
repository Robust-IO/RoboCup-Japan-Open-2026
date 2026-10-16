#!/usr/bin/env python3

import unittest
from pathlib import Path


SOURCE_FILE = (
    Path(__file__).resolve().parents[1]
    / 'src'
    / 'interactive_cleanup_sample.cpp'
)


class InteractiveCleanupBehaviorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE_FILE.read_text()

    def test_controller_uses_validated_cleanup_perception_topics(self):
        self.assertIn('/cleanup_vision/enable', self.source)
        self.assertIn('/cleanup_vision/detected_objects', self.source)
        self.assertIn('/cleanup_vision/pointing_direction', self.source)

    def test_protocol_waits_for_pick_and_destination_cues(self):
        for state_name in (
            'WaitForPickCommand',
            'ObserveTarget',
            'WaitForCleanCommand',
            'ObserveDestination',
            'MoveToObject',
            'GraspObject',
            'MoveToDestination',
            'ReleaseObject',
        ):
            self.assertIn(state_name, self.source)
        self.assertIn('received_clean_command_', self.source)
        self.assertIn('changeStep(WaitForCleanCommand);', self.source)

    def test_nav2_can_be_disabled_without_disabling_direct_motion(self):
        self.assertIn('declare_parameter<bool>("use_nav2", true)', self.source)
        self.assertIn('Nav2 disabled; using direct approach', self.source)
        self.assertIn('/hsrb/command_velocity', self.source)

    def test_target_selection_uses_temporal_and_spatial_evidence(self):
        self.assertIn('selectStablePointedTarget()', self.source)
        self.assertIn('STABLE_TARGET_MIN_FRAMES = 6', self.source)
        self.assertIn('STABLE_TARGET_MIN_CONFIDENCE = 0.25', self.source)
        self.assertIn('const std::string spatial_key', self.source)

    def test_frontal_pointing_depth_ambiguity_is_bounded(self):
        self.assertIn('forward_support == 0', self.source)
        self.assertIn('reverse_support >= STABLE_TARGET_MIN_FRAMES', self.source)
        self.assertIn('Resolving frontal pointing depth ambiguity', self.source)

    def test_floor_grasp_aligns_real_finger_collision_midpoint(self):
        self.assertIn('getGripperMidpoint(', self.source)
        self.assertIn('GRIPPER_ALIGN_STABLE_CYCLES = 3', self.source)
        self.assertIn('Fingertip midpoint aligned', self.source)
        self.assertIn('Lowered fingertip height verified', self.source)

    def test_release_uses_destination_profile_and_verified_pose(self):
        self.assertIn('PlacementProfile getPlacementProfile(', self.source)
        self.assertIn('RELEASE_ALIGN_STABLE_CYCLES = 3', self.source)
        self.assertIn('Verified placement pose', self.source)

    def test_success_messages_follow_grasp_and_release_states(self):
        self.assertIn('MSG_OBJECT_GRASPED   = "Object_grasped"', self.source)
        self.assertIn('MSG_TASK_FINISHED    = "Task_finished"', self.source)
        self.assertIn('changeStep(SendObjectGrasped);', self.source)
        self.assertIn('changeStep(SendTaskFinished);', self.source)


if __name__ == '__main__':
    unittest.main()

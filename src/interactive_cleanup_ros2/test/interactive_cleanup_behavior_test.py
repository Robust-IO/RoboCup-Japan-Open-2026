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
            'MoveToObject',
            'GraspObject',
            'MoveToDestination',
            'ReleaseObject',
        ):
            self.assertIn(state_name, self.source)
        self.assertIn('received_clean_command_', self.source)
        self.assertIn('changeStep(WaitForCleanCommand);', self.source)

    def test_gesture_recovery_never_crosses_a_command_boundary(self):
        state_machine = self.source.index('void runStateMachine()')
        target_state = self.source.index('case ObserveTarget:', state_machine)
        clean_state = self.source.index('case WaitForCleanCommand:', target_state)
        target_block = self.source[target_state:clean_state]
        self.assertNotIn('sendMessage(MSG_POINT_IT_AGAIN);', target_block)
        self.assertIn('giveUp();', target_block)
        move_state = self.source.index('case MoveToObject:', clean_state)
        clean_block = self.source[clean_state:move_state]
        self.assertNotIn('sendMessage(MSG_POINT_IT_AGAIN);', clean_block)
        self.assertNotIn('ObserveDestination', self.source)

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

    def test_target_selection_has_one_bounded_decision_path(self):
        self.assertIn('candidate.frames < STABLE_TARGET_MIN_FRAMES', self.source)
        self.assertNotIn('Multi-ray recovery selected', self.source)
        self.assertNotIn('Synchronized image-ray recovery selected', self.source)

    def test_floor_grasp_aligns_real_finger_collision_midpoint(self):
        self.assertIn('getGripperMidpoint(', self.source)
        self.assertIn('GRIPPER_ALIGN_STABLE_CYCLES = 3', self.source)
        self.assertIn('GRIPPER_TARGET_INSERTION_DEPTH = 0.010', self.source)
        self.assertIn('Finger trigger aligned', self.source)
        self.assertIn('Target-height fingertip verified', self.source)

    def test_grasp_height_is_target_dependent_and_checked_both_directions(self):
        self.assertIn('armGraspReadyForTarget()', self.source)
        self.assertIn('armGraspAtTargetHeight()', self.source)
        self.assertIn('GRIPPER_CONTACT_HEIGHT_OFFSET', self.source)
        self.assertIn('std::abs(height_error) > GRIPPER_HEIGHT_TOLERANCE', self.source)
        self.assertIn('headLookAtTarget()', self.source)
        self.assertIn('isReachableGraspHeight', self.source)

    def test_target_voting_uses_locked_gesture_across_async_object_frames(self):
        self.assertNotIn('nearest_dt > 0.50', self.source)
        self.assertIn('locked_pointing_.origin.x', self.source)
        self.assertIn('std::set<std::string> updated_in_frame', self.source)
        self.assertIn('One-shot finger-directed head aim', self.source)

    def test_target_observation_restores_successful_directed_view_and_boundary(self):
        self.assertIn('target_aim_pose_valid_', self.source)
        self.assertIn('Directed view contains no objects', self.source)
        self.assertIn(
            'Clean_up! arrived before a target was confirmed', self.source)
        self.assertIn('Ignoring out-of-phase Pick_it_up!', self.source)

    def test_clean_command_closes_destination_capture_window(self):
        self.assertIn('destination_capture_closed_ = true;', self.source)

    def test_destination_capture_starts_only_after_first_gesture_release(self):
        self.assertNotIn('elapsed() >= 0.75', self.source)
        self.assertIn(
            'DESTINATION_RELEASE_INVALID_FRAMES = 3', self.source)
        self.assertIn('destination_capture_armed_', self.source)
        self.assertIn(
            'First gesture released; destination capture armed', self.source)
        self.assertIn(
            'Destination gesture started; holding Avatar view', self.source)
        self.assertIn(
            '!destination_capture_armed_ || '
            '!destination_gesture_started_', self.source)
        state_machine = self.source.index('void runStateMachine()')
        clean_state = self.source.index(
            'case WaitForCleanCommand:', state_machine)
        move_state = self.source.index('case MoveToObject:', clean_state)
        clean_block = self.source[clean_state:move_state]
        self.assertIn('Frozen pre-Clean destination gesture', clean_block)
        self.assertIn('changeStep(WaitForResult);', clean_block)

    def test_target_confirmation_returns_camera_to_avatar(self):
        confirmation = self.source.index(
            '[ObserveTarget] Target confirmed without camera scan')
        self.assertIn('headCenter();',
                      self.source[confirmation:confirmation + 500])
        self.assertIn('POINTING_MEMORY_SEC  =  20.0', self.source)

    def test_release_uses_destination_profile_and_verified_pose(self):
        self.assertIn('PlacementProfile getPlacementProfile(', self.source)
        self.assertIn('RELEASE_ALIGN_STABLE_CYCLES = 3', self.source)
        self.assertIn('Verified placement pose', self.source)

    def test_destination_selection_uses_real_3d_trigger_and_consensus(self):
        self.assertIn('resolvePlaceDestination(', self.source)
        self.assertIn('region.center.z = profile.release_height', self.source)
        self.assertIn('pointings.size() < 10', self.source)
        self.assertIn('destination_pointing_history_', self.source)
        self.assertIn('planar_ray=', self.source)
        self.assertIn('Rejected destination gesture', self.source)

    def test_destination_uses_second_gesture_image_projection(self):
        self.assertIn('addDestinationImageEvidence(regions);', self.source)
        self.assertIn('[PointDest/Image]', self.source)
        self.assertIn('destination_pointing_history_', self.source)
        self.assertIn('image_ray=', self.source)
        self.assertIn('pt.pose_direction_valid', self.source)
        self.assertIn('"unanchored"', self.source)

    def test_drop_in_release_does_not_retry_an_unreachable_trigger_center(self):
        self.assertIn('const bool reachable_drop_pose', self.source)
        self.assertIn('placement_profile_.drop_in && height_error < 0.0',
                      self.source)
        self.assertIn('opening at lowest reachable height', self.source)

    def test_release_alignment_accepts_safe_four_centimeter_residual(self):
        self.assertIn(
            'RELEASE_ALIGN_FORWARD_TOLERANCE = 0.040', self.source)
        self.assertIn(
            'RELEASE_ALIGN_LATERAL_TOLERANCE = 0.030', self.source)

    def test_success_messages_follow_grasp_and_release_states(self):
        self.assertIn('MSG_OBJECT_GRASPED   = "Object_grasped"', self.source)
        self.assertIn('MSG_TASK_FINISHED    = "Task_finished"', self.source)
        self.assertIn('changeStep(SendObjectGrasped);', self.source)
        self.assertIn('changeStep(SendTaskFinished);', self.source)


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3

import unittest
import re
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
        self.assertIn('STABLE_TARGET_MAX_AVG_RAY_3D_M = 0.55', self.source)

    def test_destination_prefers_validated_depth_geometry(self):
        self.assertIn('pt.depth_direction_valid', self.source)
        self.assertIn('? pt.direction', self.source)

    def test_candidate_locations_cannot_reverse_observed_gesture(self):
        self.assertNotIn('reverse_support', self.source)
        self.assertIn('pointingPlaneAngle(', self.source)

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
        self.assertIn('destination_window_.close(perception_stamp_clock_.now().nanoseconds());', self.source)
        self.assertIn(
            'destination_window_.accepts(rclcpp::Time(msg->header.stamp).nanoseconds())',
            self.source)

    def test_destination_capture_uses_protocol_boundary_and_settled_view(self):
        self.assertNotIn('elapsed() >= 0.75', self.source)
        self.assertNotIn('DESTINATION_RELEASE_INVALID_FRAMES', self.source)
        self.assertNotIn('destination_capture_armed_', self.source)
        self.assertNotIn('isDistinctFromPickupGesture', self.source)
        self.assertIn(
            'destination_window_.open(perception_stamp_clock_.now().nanoseconds());', self.source)
        self.assertIn('destination_camera_ready_time_', self.source)
        self.assertIn(
            'Destination gesture captured in settled tracked view', self.source)
        state_machine = self.source.index('void runStateMachine()')
        clean_state = self.source.index(
            'case WaitForCleanCommand:', state_machine)
        move_state = self.source.index('case MoveToObject:', clean_state)
        clean_block = self.source[clean_state:move_state]
        self.assertIn('destination_pointing_history_.empty()', clean_block)
        self.assertIn('Settled pre-Clean destination gesture', clean_block)
        self.assertIn('continuing with pickup-only scoring', clean_block)
        self.assertIn('changeStep(MoveToObject);', clean_block)

    def test_avatar_tracking_continues_and_invalidates_pre_motion_gesture(self):
        tracker_start = self.source.index('bool trackAvatarWithHead()')
        tracker_end = self.source.index(
            'bool hasRecentVisibleAvatar', tracker_start)
        tracker = self.source[tracker_start:tracker_end]
        self.assertIn('AVATAR_TRACK_DEADBAND_RAD', tracker)
        self.assertIn('moveHead(target_pan, target_tilt, 0.3);', tracker)
        self.assertIn(
            'invalidatePickupPointingForHeadMotion(now_time, 0.3);',
            tracker)

        state_machine = self.source.index('void runStateMachine()')
        wait_pick = self.source.index(
            'case WaitForPickCommand:', state_machine)
        observe = self.source.index('case ObserveTarget:', wait_pick)
        wait_clean = self.source.index('case WaitForCleanCommand:', observe)
        move_object = self.source.index('case MoveToObject:', wait_clean)
        self.assertIn(
            'trackAvatarWithHead();', self.source[wait_pick:observe])
        self.assertIn(
            'reacquireAvatarForDestination();',
            self.source[wait_clean:move_object])

    def test_wait_for_pick_searches_before_gesture_when_avatar_is_missing(self):
        self.assertIn('void searchAvatarWithHead()', self.source)
        self.assertIn('[AvatarSearch] No recent person observation', self.source)
        state_machine = self.source.index('void runStateMachine()')
        wait_pick = self.source.index(
            'case WaitForPickCommand:', state_machine)
        observe = self.source.index('case ObserveTarget:', wait_pick)
        wait_block = self.source[wait_pick:observe]
        self.assertIn('hasRecentVisibleAvatar()', wait_block)
        self.assertIn('trackAvatarWithHead();', wait_block)
        self.assertIn('searchAvatarWithHead();', wait_block)

    def test_pickup_ray_is_accepted_after_settled_person_tracking(self):
        self.assertIn('pointing_camera_ready_time_', self.source)
        self.assertIn('pickup_camera_settled', self.source)
        self.assertIn('pickup_has_recent_person', self.source)
        self.assertNotIn('pickup_person_centered', self.source)
        self.assertIn(
            'void invalidatePickupPointingForHeadMotion(', self.source)
        invalidator_start = self.source.index(
            'void invalidatePickupPointingForHeadMotion(')
        tracker_start = self.source.index(
            'bool trackAvatarWithHead()', invalidator_start)
        invalidator = self.source[invalidator_start:tracker_start]
        self.assertIn('has_last_valid_pointing_ = false;', invalidator)
        self.assertIn('pickup_pointing_history_.clear();', invalidator)
        self.assertIn('obs_pointings_.clear();', invalidator)

    def test_avatar_search_leaves_a_real_settled_observation_window(self):
        def constant(name):
            match = re.search(
                rf'{name}\s*=\s*([0-9]+(?:\.[0-9]+)?)', self.source)
            self.assertIsNotNone(match, name)
            return float(match.group(1))

        search_interval = constant('AVATAR_SEARCH_INTERVAL_SEC')
        search_motion = constant('AVATAR_SEARCH_MOTION_SEC')
        settle = constant('DESTINATION_HEAD_SETTLE_SEC')
        self.assertGreater(
            search_interval, search_motion + settle,
            'search restarts before the camera can produce a settled ray')

        close_interval = constant('CLOSE_GESTURE_SEARCH_INTERVAL_SEC')
        close_motion = constant('CLOSE_GESTURE_SEARCH_MOTION_SEC')
        self.assertGreater(
            close_interval, close_motion + settle,
            'close-person sweep restarts before its settled observation window')

    def test_visible_close_avatar_gets_bounded_gesture_search(self):
        self.assertIn('void searchGestureAroundVisibleAvatar()', self.source)
        self.assertIn('[CloseGestureSearch]', self.source)
        state_machine = self.source.index('void runStateMachine()')
        wait_pick = self.source.index(
            'case WaitForPickCommand:', state_machine)
        observe = self.source.index('case ObserveTarget:', wait_pick)
        wait_block = self.source[wait_pick:observe]
        self.assertIn('searchGestureAroundVisibleAvatar();', wait_block)
        self.assertIn('hasRecentRawPointing()', wait_block)
        search_start = self.source.index(
            'void searchGestureAroundVisibleAvatar()')
        search_end = self.source.index(
            'void searchAvatarWithHead()', search_start)
        close_search = self.source[search_start:search_end]
        self.assertIn('last_person_pixel_x_', close_search)
        self.assertIn('last_person_pixel_y_', close_search)
        self.assertIn('current_pan - image_yaw', close_search)

    def test_destination_tracking_preserves_captured_world_geometry(self):
        self.assertIn('personOutsideGestureCaptureEnvelope()', self.source)
        reacquire_start = self.source.index(
            'void reacquireAvatarForDestination()')
        reacquire_end = self.source.index(
            '// =========================================================================\n'
            '  // Gripper', reacquire_start)
        reacquire = self.source[reacquire_start:reacquire_end]
        self.assertIn('const bool tracked = trackAvatarWithHead();', reacquire)
        self.assertIn('if (!tracked && !hasRecentRawPointing())', reacquire)
        self.assertNotIn('destination_pointing_history_.clear()', reacquire)

    def test_short_pose_only_ray_cannot_be_locked_as_pickup(self):
        lock_start = self.source.index('bool lockRecentPointing()')
        lock_end = self.source.index(
            'bool hasRecentImagePointing', lock_start)
        lock = self.source[lock_start:lock_end]
        self.assertIn('image_length < MIN_PICKUP_IMAGE_SEGMENT', lock)
        self.assertIn('!pt.depth_direction_valid', lock)
        self.assertIn('pt.depth_direction_samples < 4', lock)
        self.assertIn('Frontal gesture aimed from measured RGB-D ray', self.source)

    def test_single_negative_pose_frame_does_not_erase_avatar_track(self):
        callback_start = self.source.index('void pointingCallback(')
        callback_end = self.source.index(
            '// =========================================================================\n'
            '  // Joint helpers', callback_start)
        callback = self.source[callback_start:callback_end]
        self.assertIn('if (msg->person_is_valid && capturedAfterHeadMotion(msg->header.stamp)) {', callback)
        self.assertIn('has_person_observation_ = true;', callback)
        self.assertNotIn('has_person_observation_ = false;', callback)

        recent_start = self.source.index('bool hasRecentVisibleAvatar(')
        recent_end = self.source.index(
            'void searchAvatarWithHead()', recent_start)
        recent = self.source[recent_start:recent_end]
        self.assertIn('has_person_observation_', recent)
        self.assertIn('last_person_observation_time_', recent)

    def test_destination_world_vote_survives_camera_tracking_motion(self):
        self.assertIn('DESTINATION_HEAD_SETTLE_SEC = 0.35', self.source)
        self.assertIn('destination_camera_settled', self.source)
        self.assertIn('destination_camera_ready_time_', self.source)
        self.assertIn(
            '[WaitForClean] Following moving Avatar; resetting destination image samples',
            self.source)
        tracker_start = self.source.index('bool trackAvatarWithHead()')
        tracker_end = self.source.index(
            'bool hasRecentVisibleAvatar', tracker_start)
        tracker = self.source[tracker_start:tracker_end]
        self.assertIn('destination_image_pointing_history_.clear();', tracker)
        self.assertNotIn('destination_pointing_history_.clear();', tracker)
        self.assertNotIn('destination_gesture_started_ = false;', tracker)

    def test_destination_failure_does_not_discard_pickup_score(self):
        state_machine = self.source.index('void runStateMachine()')
        grasped_state = self.source.index(
            'case SendObjectGrasped:', state_machine)
        destination_state = self.source.index(
            'case MoveToDestination:', grasped_state)
        grasped_block = self.source[grasped_state:destination_state]
        self.assertIn('sendMessage(MSG_OBJECT_GRASPED);', grasped_block)
        self.assertIn('if (dest_valid_)', grasped_block)
        self.assertIn('preserving grasp before Give_up', grasped_block)
        self.assertIn('sendMessage(MSG_GIVE_UP);', grasped_block)
        self.assertLess(
            grasped_block.index('sendMessage(MSG_OBJECT_GRASPED);'),
            grasped_block.index('sendMessage(MSG_GIVE_UP);'))

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

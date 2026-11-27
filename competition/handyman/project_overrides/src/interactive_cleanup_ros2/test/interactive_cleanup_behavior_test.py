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

    def test_controller_uses_new_perception_topics(self):
        self.assertIn('/cleanup_perception/mode', self.source)
        self.assertIn('/cleanup_perception/head/avatar', self.source)
        self.assertIn('/cleanup_perception/head/objects', self.source)
        self.assertIn('/cleanup_perception/head/pointing', self.source)
        self.assertIn('/cleanup_perception/hand/target_alignment', self.source)

    def test_state_machine_uses_new_resolution_and_approach_phases(self):
        for state_name in (
            'WaitForPickTrackAvatar',
            'ResolvePickTarget',
            'WaitForCleanTrackAvatar',
            'ResolvePlaceDestination',
            'PlanPregrasp',
            'NavigateToPregrasp',
            'DeployArmForApproach',
            'HandCameraApproach',
            'CloseAndVerifyGrasp',
        ):
            self.assertIn(state_name, self.source)
        self.assertNotIn('RecoverAvatarViewAfterPick', self.source)

    def test_old_head_scan_flow_is_removed(self):
        self.assertNotIn('beginObservationScan', self.source)
        self.assertNotIn('headScanStep(', self.source)
        self.assertNotIn('SCAN_LOCAL', self.source)
        self.assertNotIn('SCAN_WIDE', self.source)

    def test_wait_states_track_avatar_with_small_yaw_corrections(self):
        self.assertIn('computeAvatarTrackCommand(', self.source)
        self.assertIn('TRACK_AVATAR', self.source)

    def test_pick_resolution_uses_bounded_head_micro_sweep_after_base_alignment(self):
        resolve_pick = self._extract_case_block(
            'ResolvePickTarget', 'WaitForCleanTrackAvatar')
        self.assertIn('beginObservationAlignment("ResolvePickTarget"', resolve_pick)
        self.assertIn('runPointingAlignment("ResolvePickTarget")', resolve_pick)
        self.assertIn('beginPickHeadSweep("ResolvePickTarget")', resolve_pick)
        self.assertIn('runPickHeadSweep("ResolvePickTarget")', resolve_pick)
        self.assertIn('PICK_HEAD_SWEEP_TILT = -0.20', self.source)

    def test_pick_reobservation_alignment_can_reuse_latched_pick_pointings(self):
        alignment_helper = self._extract_function_block(
            'std::vector<interactive_cleanup::PointingObservation> collectRecentAlignmentPointings() const')
        self.assertIn('observe_align_seed_pointings_', alignment_helper)
        self.assertIn('latched_pick_pointings_', self.source)

    def test_active_pick_resolution_merges_latched_and_reobserved_pointings(self):
        merge_helper = self._extract_function_block(
            'std::vector<TimedPointingSample> collectPickResolutionPointings() const')
        self.assertIn('latched_pick_pointings_', merge_helper)
        self.assertIn('obs_pointings_', merge_helper)
        resolve_pick = self._extract_case_block(
            'ResolvePickTarget', 'ResolvePlaceDestination')
        self.assertIn('resolvePickFromObservations(', resolve_pick)
        self.assertIn('obs_objects_', resolve_pick)
        self.assertIn('collectPickResolutionPointings()', resolve_pick)

    def test_avatar_cues_are_latched_before_resolution(self):
        self.assertIn('CUE_CAPTURE_WINDOW_SEC', self.source)
        self.assertIn('latchPickCueFromRecentObservations(', self.source)
        self.assertIn('latchPlaceCueFromRecentObservations(', self.source)
        self.assertIn('pruneCueCaptureWindow(', self.source)
        self.assertIn('latched_pick_objects_', self.source)
        self.assertIn('latched_place_objects_', self.source)

    def test_cleanup_flow_waits_for_both_latched_cues_before_resolving(self):
        self.assertIn('changeStep(WaitForCleanTrackAvatar);', self.source)
        self.assertIn('changeStep(ResolvePickTarget);', self.source)
        self.assertIn('changeStep(ResolvePlaceDestination);', self.source)
        self.assertNotIn('pre_pick_avatar_yaw_', self.source)
        self.assertNotIn('changeStep(RecoverAvatarViewAfterPick);', self.source)

    def test_target_and_destination_resolution_use_pure_resolvers(self):
        self.assertIn('resolvePickTarget(', self.source)
        self.assertIn('resolvePlaceDestination(', self.source)
        self.assertIn('loadDestinationRegions(', self.source)

    def test_grasp_path_uses_pregrasp_planner_and_hand_servo(self):
        self.assertIn('planPregrasp(', self.source)
        self.assertIn('computeHandServoCommand(', self.source)
        self.assertIn('shouldRetryHandApproach(', self.source)

    def test_hand_camera_approach_combines_base_and_arm_vertical_servo(self):
        hand_approach = self._extract_case_block(
            'HandCameraApproach', 'CloseAndVerifyGrasp')
        self.assertIn('moveBase(command.linear_x, command.linear_y, 0.0);', hand_approach)
        self.assertIn('applyHandServoLiftDelta(command.lift_delta);', hand_approach)

    def test_object_grasped_transition_is_gated_by_verification(self):
        self.assertIn('if (grasp_verified_)', self.source)
        self.assertIn('changeStep(SendObjectGrasped);', self.source)

    def _extract_case_block(self, case_name: str, next_case_name: str) -> str:
        start = self.source.index(f'case {case_name}: {{')
        end = self.source.index(f'case {next_case_name}: {{', start)
        return self.source[start:end]

    def _extract_function_block(self, signature: str) -> str:
        start = self.source.index(signature)
        end = self.source.index('\n\n  void ', start)
        return self.source[start:end]


if __name__ == '__main__':
    unittest.main()

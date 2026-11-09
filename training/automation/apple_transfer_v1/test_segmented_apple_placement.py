import copy
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from apple_transfer_auto_backend import LiveBackend
from apple_transfer_auto_plan import placement_step
from apple_transfer_automation import STAGES
from continue_apple_placement import validate_source


def snapshot(y=2.525):
    return dict(streams=dict(depth=dict(stamp_ns=1)),tracking_transforms=dict(map_from_base_at_depth=dict(
        stamp_ns=1,source_frame='base_link',target_frame='map',translation=[2.1,y,0],
        quaternion=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)])))


class SegmentTest(unittest.TestCase):
    def test_long_approach_is_bounded_segment_not_wider_command(self):
        step=placement_step(dict(center_base=[.707,.078,.494]),snapshot(),dict(release_xy_map_m=[1.971,3.65]))
        self.assertFalse(step['aligned']);self.assertAlmostEqual(math.hypot(*step['delta_m'][:2]),.32)
    def test_too_far_or_backward_rejected(self):
        for target in ([1.97,5.], [1.97,3.]):
            with self.assertRaises(ValueError):placement_step(dict(center_base=[.707,.078,.494]),snapshot(),dict(release_xy_map_m=target))
    def test_actual_adapter_remeasures_after_successful_segments(self):
        with tempfile.TemporaryDirectory() as temp:
            b=LiveBackend.__new__(LiveBackend);b.output=Path(temp);b.holding=.395
            b.config=dict(policy=dict(placement_approach_mode='segmented'))
            b.place_plan=dict(release_xy_map_m=[2.022,3.65],palm_height_m=.484)
            state=snapshot();b.latest=Mock(side_effect=lambda *args:(Path(temp),copy.deepcopy(state),None,None))
            b.held_measurement=Mock(return_value=dict(center_base=[.707,.078,.494]))
            def adjust(name,pose,delta,arm,opening):
                self.assertLessEqual(math.hypot(*delta[:2]),.4)
                self.assertEqual(opening,.395)
                state['tracking_transforms']['map_from_base_at_depth']['translation'][0]-=delta[1]
                state['tracking_transforms']['map_from_base_at_depth']['translation'][1]+=delta[0]
                return name+'.json'
            b.adjust=Mock(side_effect=adjust)
            self.assertTrue(b.approach_placement()['verified'])
            self.assertEqual(b.adjust.call_count,2)
            self.assertEqual(b.held_measurement.call_count,3)

    def test_single_mode_keeps_the_entire_approach(self):
        m=dict(center_base=[.707,.078,.494]);s=snapshot();p=dict(release_xy_map_m=[1.971,3.65])
        step=placement_step(m,s,p,single_move=True)
        self.assertGreater(math.hypot(*step['delta_m'][:2]),.4)
        self.assertAlmostEqual(math.hypot(*step['delta_m'][:2]),step['distance_m'])

    def test_single_mode_full_distance_not_fixed_cap(self):
        step=placement_step(dict(center_base=[.707,.078,.494]),snapshot(),dict(release_xy_map_m=[1.97,5.]),single_move=True)
        self.assertGreater(step['distance_m'],1.)
        self.assertAlmostEqual(math.hypot(*step['delta_m'][:2]),step['distance_m'])

    def test_single_adapter_sends_one_command_and_remeasures_without_split(self):
        with tempfile.TemporaryDirectory() as temp:
            b=LiveBackend.__new__(LiveBackend);b.output=Path(temp);b.holding=.395
            b.config=dict(policy=dict(placement_approach_mode='single'))
            b.place_plan=dict(release_xy_map_m=[2.022,3.65],palm_height_m=.484)
            state=snapshot();b.latest=Mock(side_effect=lambda *args:(Path(temp),copy.deepcopy(state),None,None))
            b.held_measurement=Mock(return_value=dict(center_base=[.707,.078,.494]))
            def adjust(name,pose,delta,arm,opening,*,single_placement_approach):
                self.assertTrue(single_placement_approach)
                self.assertGreater(math.hypot(*delta[:2]),.4)
                state['tracking_transforms']['map_from_base_at_depth']['translation'][0]-=delta[1]
                state['tracking_transforms']['map_from_base_at_depth']['translation'][1]+=delta[0]
                return name+'.json'
            b.adjust=Mock(side_effect=adjust)
            self.assertTrue(b.approach_placement()['verified'])
            self.assertEqual(b.adjust.call_count,1)
            self.assertEqual(b.held_measurement.call_count,2)

    def test_single_mode_never_sends_automatic_correction_if_endpoint_not_aligned(self):
        with tempfile.TemporaryDirectory() as temp:
            b=LiveBackend.__new__(LiveBackend);b.output=Path(temp);b.holding=.395
            b.config=dict(policy=dict(placement_approach_mode='single'))
            b.place_plan=dict(release_xy_map_m=[2.022,3.65],palm_height_m=.484)
            b.latest=Mock(return_value=(Path(temp),snapshot(),None,None))
            b.held_measurement=Mock(return_value=dict(center_base=[.707,.078,.494]))
            b.adjust=Mock(return_value='one-command.json')
            with self.assertRaisesRegex(ValueError,'single_placement_endpoint_not_aligned'):
                b.approach_placement()
            self.assertEqual(b.adjust.call_count,1)

    def test_single_mode_still_rejects_invalid_targets(self):
        for target in ([float('nan'),5.],[1.97,float('inf')],[1.97,3.]):
            with self.assertRaises(ValueError):
                placement_step(dict(center_base=[.707,.078,.494]),snapshot(),dict(release_xy_map_m=target),single_move=True)
    def context(self):
        result=dict(status='stopped',stage='approach_placement',confirmed_stages=list(STAGES[:STAGES.index('approach_placement')]),placement_verified=False,task_id='old')
        return [result,[],dict(task_id='old',destination_id='square_low_table#living_room'),dict(status='passed',inverse_place_stage='prepare'),dict(grasp_success_verified=True)]
    def test_only_audited_precommand_context_can_continue(self):self.assertTrue(validate_source(*self.context()))
    def test_any_earlier_placement_intent_blocks_replay(self):
        for stage in ('approach_placement','release','withdraw'):
            args=self.context();args[1].append(dict(event='command_intent',stage=stage))
            with self.subTest(stage=stage),self.assertRaises(ValueError):validate_source(*args)
    def test_other_stop_stage_cannot_skip_grasp_or_repeat_release(self):
        args=self.context();args[0]['stage']='release'
        with self.assertRaises(ValueError):validate_source(*args)


if __name__=='__main__':unittest.main()

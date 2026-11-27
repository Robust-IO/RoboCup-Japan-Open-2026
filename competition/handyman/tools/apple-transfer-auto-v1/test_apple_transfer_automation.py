import contextlib
import copy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from apple_transfer_auto_plan import (clear_patch, fresh_snapshot, map_pose, plan_grasp,
                                     placement_delta, validate_config)
from apple_transfer_automation import RobotLock, STAGES, run
from inspect_transfer_ready import validate_feedback

CONFIG = json.loads((Path(__file__).parent/'config/apple_transfer_auto_v1.json').read_text())


class FakeBackend:
    mode = 'offline_test'
    def __init__(self, fail=None, unverified=None):
        self.fail, self.unverified = fail, unverified
        self.calls, self.closed = [], False
    def bind(self, output, task_id, record):
        self.record = record
    def perform(self, stage):
        self.record(dict(event='test_observed_intent', stage=stage))
        self.calls.append(stage)
        if stage == self.fail:
            raise RuntimeError('injected_failure')
        return dict(verified=stage != self.unverified, evidence='SYNTHETIC_TEST_ONLY')
    def close(self):
        self.closed = True


class CoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.output = Path(self.temp.name)/'new-run'
    def tearDown(self):
        self.temp.cleanup()
    def execute(self, backend):
        with contextlib.redirect_stdout(io.StringIO()):
            return run(CONFIG, 'test-task', self.output, backend)
    def test_complete_offline_flow_never_claims_live(self):
        backend = FakeBackend()
        result = self.execute(backend)
        self.assertEqual(result['status'], 'offline_test_complete')
        self.assertEqual(backend.calls, list(STAGES))
        self.assertTrue(backend.closed)
        self.assertFalse(result['task_finished_sent'])
        self.assertFalse(result['does_not_exist_sent'])
    def test_every_failure_stops_later_stages(self):
        for index, stage in enumerate(STAGES):
            with self.subTest(stage=stage):
                self.output = Path(self.temp.name)/stage
                backend = FakeBackend(fail=stage)
                result = self.execute(backend)
                self.assertEqual(result['status'], 'stopped')
                self.assertEqual(backend.calls, list(STAGES[:index+1]))
                self.assertFalse(result['automatic_resume'])
                self.assertTrue(backend.closed)
    def test_failed_grasp_cannot_navigate_or_release(self):
        backend = FakeBackend(unverified='verify_grasp')
        self.execute(backend)
        self.assertNotIn('navigate_destination', backend.calls)
        self.assertNotIn('release', backend.calls)
    def test_release_failure_never_withdraws_or_releases_again(self):
        backend = FakeBackend(fail='release')
        self.execute(backend)
        self.assertEqual(backend.calls.count('release'), 1)
        self.assertNotIn('withdraw', backend.calls)
    def test_same_output_cannot_replay_completed_run(self):
        self.execute(FakeBackend())
        next_backend = FakeBackend()
        with self.assertRaises(FileExistsError):
            self.execute(next_backend)
        self.assertEqual(next_backend.calls, [])
    def test_intents_are_durable_before_adapter_actions(self):
        self.execute(FakeBackend())
        records = [json.loads(s) for s in (self.output/'journal.jsonl').read_text().splitlines()]
        for index, row in enumerate(records):
            if row['event'] == 'test_observed_intent':
                self.assertEqual(records[index-1]['event'], 'intent')
                self.assertEqual(records[index-1]['stage'], row['stage'])
    def test_interrupted_release_is_not_replayed(self):
        class Interrupted(FakeBackend):
            def perform(self, stage):
                if stage == 'release':
                    self.calls.append(stage)
                    raise KeyboardInterrupt()
                return super().perform(stage)
        backend = Interrupted()
        result = self.execute(backend)
        self.assertEqual(result['reason'], 'interrupted')
        self.assertNotIn('withdraw', backend.calls)
        with self.assertRaises(FileExistsError):
            self.execute(FakeBackend())
    def test_global_lock_serializes_different_output_paths(self):
        lock = str(Path(self.temp.name)/'robot.lock')
        with RobotLock(lock):
            with self.assertRaises(RuntimeError):
                with RobotLock(lock):
                    self.fail('two automation owners')
        with RobotLock(lock):
            pass
    def test_placement_continuation_requires_reviewed_context(self):
        with self.assertRaises(ValueError):
            run(CONFIG,'test-task',self.output,FakeBackend(),placement_only=True)
    def test_partial_continuation_never_claims_full_automatic_transfer(self):
        backend=FakeBackend();backend.mode='live';backend.placement_continuation_authorized=True
        with contextlib.redirect_stdout(io.StringIO()):
            result=run(CONFIG,'test-task',self.output,backend,placement_only=True)
        self.assertEqual(result['status'],'reviewed_placement_continuation_verified')
        self.assertEqual(result['run_kind'],'reviewed_placement_continuation')
        self.assertNotIn('grasp_and_lift',backend.calls)
        self.assertFalse(result['task_finished_sent'])


class PlanningTest(unittest.TestCase):
    def test_no_navigation_parameter_changes_or_protocol_enable(self):
        for key, value in [('navigation_tolerances', [.2, .2, .2]),
                           ('layout', 'LayoutB'), ('execution', dict(auto_protocol_success=True, automatic_action_retry=False))]:
            config = copy.deepcopy(CONFIG)
            config[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_config(config)
    def test_nan_policy_rejected(self):
        config = copy.deepcopy(CONFIG)
        config['policy']['release_gap_m'] = float('nan')
        with self.assertRaises(ValueError):
            validate_config(config)
    def test_edge_gap_and_uncertainty_cannot_be_silently_lowered(self):
        for key, value in [('edge_margin_m', .01), ('edge_uncertainty_m', 0.)]:
            config = copy.deepcopy(CONFIG)
            config['policy'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_config(config)
    def snapshot(self):
        return dict(tracking_epoch='epoch', streams=dict(rgb=dict(stamp_ns=1_000_000_000), depth=dict(stamp_ns=1_000_000_000)),
                    joint_stamp_ns=1_000_000_000, tracking_transforms=dict(map_from_base_at_depth=dict(stamp_ns=1_000_000_000,
                    translation=[10.15, -6.61, 0], quaternion=[0,0,0,1], source_frame='base_link', target_frame='map')))
    def test_expired_or_wrong_epoch_rgbd_rejected(self):
        for now, epoch in [(1_500_000_000, 'epoch'), (1_200_000_000, 'old')]:
            with self.assertRaises(ValueError):
                fresh_snapshot(self.snapshot(), now, epoch)
    def test_wrong_image_time_tf_rejected(self):
        snapshot = self.snapshot()
        snapshot['tracking_transforms']['map_from_base_at_depth']['stamp_ns'] += 1
        with self.assertRaises(ValueError):
            map_pose(snapshot)
    def grasp_fixture(self):
        table = dict(id='white_side_table#bedroom_1', axis_aligned_bounds=dict(center=dict(x=11., y=-6.4), size=dict(x=.72, y=1.56)))
        evidence = dict(target_base_m=[.704, .078, .539], stamp_ns=1_000_000_000)
        observation = dict(identity_tracking=dict(identity_confirmed=True), tracking_proposals=[dict(detection=dict(name='apple'),
            candidate=dict(stamp_ns=1_000_000_000, radius_m=.032), support=dict(support_consistent=True, support_top_midpoint_z_m=.53))])
        return observation, self.snapshot(), table, evidence, CONFIG
    def test_grasp_height_is_measured_not_previous_trial_constant(self):
        args = self.grasp_fixture()
        plan = plan_grasp(*args)
        self.assertAlmostEqual(plan['palm_height_m'], .53)
        args[0]['tracking_proposals'][0]['support']['support_top_midpoint_z_m'] = .54
        self.assertAlmostEqual(plan_grasp(*args)['contact_start_lift_m']-plan['contact_start_lift_m'], .01)
    def test_wrong_source_table_target_rejected(self):
        args = self.grasp_fixture()
        args[3]['target_base_m'][1] = 2
        with self.assertRaises(ValueError):
            plan_grasp(*args)
    def test_unknown_or_occupied_release_patch_rejected(self):
        x, y = np.meshgrid(np.linspace(-.085,.085,60), np.linspace(-.085,.085,60))
        points = np.c_[x.ravel(), y.ravel(), np.full(x.size, .44)]
        self.assertTrue(clear_patch(points, [0,0], [0,0,.44], .09)['clear'])
        self.assertFalse(clear_patch(np.r_[points, [[0,0,.49]]], [0,0], [0,0,.44], .09)['clear'])
        self.assertFalse(clear_patch(points[points[:,0]>.005], [0,0], [0,0,.44], .09)['clear'])
    def test_placement_does_not_automatically_split_unreviewed_large_step(self):
        with self.assertRaises(ValueError):
            placement_delta(dict(center_base=[.7,0,.5]), self.snapshot(), dict(release_xy_map_m=[20,-6.5]))


class ReadinessTest(unittest.TestCase):
    def fixture(self):
        return [dict(arm_lift_joint=0., arm_flex_joint=0., arm_roll_joint=0., wrist_flex_joint=-math.pi/2,
                     wrist_roll_joint=0., hand_motor_joint=.65), [.06,.08,.06], {'a':0}, {'a':1}, 1]
    def test_idle_empty_tucked_robot(self):
        self.assertTrue(validate_feedback(*self.fixture())['ready'])
    def test_holding_or_other_controller_or_wrong_precision_blocks(self):
        for kind in ('holding', 'publisher', 'precision', 'active_navigation', 'extended'):
            args = self.fixture()
            if kind == 'holding': args[0]['hand_motor_joint'] = .415
            if kind == 'publisher': args[2]['a'] = 1
            if kind == 'precision': args[1][0] = .2
            if kind == 'active_navigation': args[4] = 3
            if kind == 'extended': args[0]['arm_flex_joint'] = -1.7
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                validate_feedback(*args)


if __name__ == '__main__':
    unittest.main()

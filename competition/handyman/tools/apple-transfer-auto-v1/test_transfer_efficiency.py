"""Synthetic non-navigation tests; no ROS, model or robot connection."""
import copy
import json
import math
from pathlib import Path
import socketserver
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from transfer_efficiency import wait_ready, burst_complete, coalesce_wrist_flex
from apple_shared_inference import SharedWorker, make_handler, registry_hash, rpc
from inverse_table_place_plan import build
from run_bedroom_apple_regrasp import forward_approach_delta
from reviewed_carry_adjustment import single_grasp_limit
from apple_transfer_auto_plan import validate_config
from apple_transfer_auto_backend import LiveBackend
import test_apple_approach_lease as lease_fixture


class ReadinessTests(unittest.TestCase):
    def clock_setup(self):
        self.now = 0.
        def pump(dt): self.now += dt
        return pump, lambda: self.now

    def test_ready_does_not_wait_four_seconds(self):
        pump, clock = self.clock_setup()
        wait_ready(lambda: True, pump, clock=clock)
        self.assertLess(self.now, .2)

    def test_waits_for_data_and_stable_connection(self):
        pump, clock = self.clock_setup()
        wait_ready(lambda: self.now > .5, pump, clock=clock)
        self.assertGreaterEqual(self.now, .65)
        self.assertLess(self.now, .75)

    def test_missing_data_does_not_authorize(self):
        pump, clock = self.clock_setup()
        with self.assertRaisesRegex(RuntimeError, 'readiness_timeout'):
            wait_ready(lambda: False, pump, timeout=1., clock=clock)

    def test_wrong_state_is_not_retried(self):
        pump, clock = self.clock_setup()
        def check(): raise RuntimeError('another_controller_present')
        with self.assertRaisesRegex(RuntimeError, 'another_controller_present'):
            wait_ready(check, pump, clock=clock)
        self.assertEqual(self.now, 0)

    def test_source_time_span_and_distinct_frames_required(self):
        self.assertFalse(burst_complete([1_000_000_000]*3, 3, 1.2))
        self.assertFalse(burst_complete([1_000_000_000,1_500_000_000,2_000_000_000],3,1.2))
        self.assertTrue(burst_complete([1_000_000_000,1_600_000_000,2_200_000_000],3,1.2))
        self.assertFalse(burst_complete([3,2,1],3,0))


class CoordinatedArmTests(unittest.TestCase):
    def test_prepare_preserves_high_lift_final_height_and_grip(self):
        old = build('prepare', .485, .4)
        before = copy.deepcopy(old)
        steps = coalesce_wrist_flex(old['initial'], old['steps'])
        self.assertEqual(old, before)
        self.assertEqual(len(steps), 3)
        self.assertEqual(steps[0], old['steps'][0])
        self.assertEqual(steps[-1], old['steps'][-1])
        self.assertTrue(steps[1]['coordinated_wrist_flex'])
        self.assertTrue(all(s['goal'][5] == .4 for s in steps))

    def test_retract_keeps_lowering_after_fold(self):
        old = build('retract', .485, .4)
        steps = coalesce_wrist_flex(old['initial'], old['steps'])
        self.assertEqual(len(steps), 2)
        self.assertEqual(steps[0]['goal'][0], old['initial'][0])
        self.assertEqual(steps[-1]['goal'], old['steps'][-1]['goal'])

    def test_release_and_raise_are_never_merged(self):
        for name in ('release','raise'):
            plan = build(name, .485, .4)
            self.assertEqual(coalesce_wrist_flex(plan['initial'],plan['steps']),plan['steps'])

    def test_nonconstant_palm_orientation_not_coalesced(self):
        plan=build('prepare',.485,.4)
        plan['steps'][2]['goal'][1] = -1.3
        self.assertEqual(coalesce_wrist_flex(plan['initial'],plan['steps']),plan['steps'])

    def test_single_full_delta_not_split_into_22cm(self):
        delta=forward_approach_delta([1.0521353608692117,-.019570358793591255,.536],.53,single=True)
        self.assertGreater(math.hypot(*delta), .35)
        self.assertAlmostEqual(delta[0], 1.0521353608692117-.704)
        self.assertLess(math.hypot(*forward_approach_delta([1.052,-.0195,.536],.53)),.221)

    def test_unlimited_grasp_is_only_for_measured_empty_horizontal_pose(self):
        arm=[.2527,-1.7,0,.1292036732051034,0]
        self.assertIsNone(single_grasp_limit([.8,0,0],arm,.65,'current'))
        for delta,opening,recording in [([-.8,0,0],.65,'current'),([.8,0,0],.4,'current'),([.8,0,0],.65,None)]:
            with self.assertRaises(ValueError):single_grasp_limit(delta,arm,opening,recording)

    def test_full_approach_retains_fresh_identity_and_no_overshoot(self):
        fixture=lease_fixture.Tests();fixture.setUp()
        fixture.observation['identity_tracking']['measurement']['point_odom_m'][0]=1.204
        fixture.kw.update(translation=[.5,0],max_step_m=None,single_approach=True,min_forward_gap_m=.01)
        self.assertGreater(fixture.check()['remaining_target_minus_contact_xy_m'][0], .01)
        fixture.kw['translation']=[.52,0]
        with self.assertRaises(ValueError):fixture.check()
        fixture.kw.update(translation=[.5,0],now_ns=2_000_000_000)
        with self.assertRaises(ValueError):fixture.check()

    def test_legacy_none_limit_is_not_implicitly_enabled(self):
        fixture=lease_fixture.Tests();fixture.setUp();fixture.kw['max_step_m']=None
        with self.assertRaises(ValueError):fixture.check()


class SharedModelTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.root=Path(self.directory.name)
        self.registry=self.root/'registry.json';self.registry.write_text(json.dumps(dict(test_only=True,sha256='0'*64)))
        self.endpoint=self.root/'model.sock'
        self.worker=SimpleNamespace(ready=dict(status='ready',imgsz=960,near_grasp_roi_enabled=True,model_sha256='0'*64),
            infer=Mock(return_value=dict(status='rejected',detections=[],motion_authorized=False)))
        self.server=socketserver.UnixStreamServer(str(self.endpoint),
            make_handler(self.worker,self.root,registry_hash(self.registry)))
        self.thread=threading.Thread(target=self.server.serve_forever,kwargs=dict(poll_interval=.01))
        self.thread.start()

    def tearDown(self):
        self.server.shutdown();self.thread.join(timeout=2);self.server.server_close();self.directory.cleanup()

    def test_two_clients_share_worker_and_do_not_close_owner(self):
        first=SharedWorker(self.endpoint,self.registry,confirm_apple_roi=True)
        first.infer(self.root/'source');first.close()
        second=SharedWorker(self.endpoint,self.registry)
        self.assertTrue(second.ready['shared_model'])
        second.infer(self.root/'placed');second.close()
        self.assertEqual(self.worker.infer.call_count,2)
        self.assertTrue(self.worker.infer.call_args_list[0].kwargs['confirm_apple_roi'])
        self.assertFalse(self.worker.infer.call_args_list[1].kwargs['confirm_apple_roi'])

    def test_wrong_model_context_rejected(self):
        self.registry.write_text(json.dumps(dict(different=True,sha256='1'*64)))
        with self.assertRaisesRegex(ValueError,'context_mismatch'):SharedWorker(self.endpoint,self.registry)

    def test_outside_run_capture_rejected_before_model_execution(self):
        client=SharedWorker(self.endpoint,self.registry)
        with self.assertRaisesRegex(RuntimeError,'outside_owned_transfer'):client.infer(self.root.parent/'outside')
        self.worker.infer.assert_not_called()

    def test_worker_failure_does_not_retry_inference(self):
        self.worker.infer.side_effect=RuntimeError('worker_exited')
        client=SharedWorker(self.endpoint,self.registry)
        with self.assertRaisesRegex(RuntimeError,'worker_exited'):client.infer(self.root/'frame')
        self.assertEqual(self.worker.infer.call_count,1)


class WiringTests(unittest.TestCase):
    def test_strict_opt_in(self):
        config=json.loads((Path(__file__).parent/'config/apple_transfer_auto_v1.json').read_text())
        for value in ('true',1,None):
            config['policy']['efficient_non_navigation']=value
            with self.assertRaises(ValueError):validate_config(config)

    def test_final_capture_keeps_three_source_frames_and_span(self):
        backend=LiveBackend.__new__(LiveBackend)
        backend.config=dict(policy=dict(efficient_non_navigation=True))
        backend.code=Path('/offline');backend.output=Path('/offline');backend.task_id='test'
        backend.command=Mock()
        with patch('apple_transfer_auto_backend.Path.glob',return_value=[Path('/offline/frame/snapshot.json')]):
            backend.capture('placed-final')
        args=backend.command.call_args.args[0]
        self.assertEqual(args[args.index('--stop-after-frames')+1],'3')
        self.assertEqual(args[args.index('--min-frame-span')+1],'1.2')


if __name__=='__main__':unittest.main()

import unittest
from types import SimpleNamespace
from motion_stop import MotionGate,stationary_proof
class StopTests(unittest.TestCase):
    def test_gate_blocks_new_commands_preserves_finite_deadline(self):
        sent=[];g=MotionGate(clock=lambda:10)
        m=SimpleNamespace(points=[SimpleNamespace(time_from_start=SimpleNamespace(sec=2,nanosec=0))])
        g.publish(sent.append,m);self.assertEqual(g.deadline,12)
        g.cancel()
        with self.assertRaises(RuntimeError):g.publish(sent.append,m)
        self.assertEqual(len(sent),1)
    def states(self):return [dict(stamp_ns=t,joints=[0,0,0,0,0,.3],poses={'odom':[1,2,0]}) for t in [1,200000001,400000001]]
    def test_actual_stationary_feedback(self):self.assertTrue(stationary_proof(self.states())['motion_stopped'])
    def test_process_exit_or_one_frame_is_not_proof(self):
        with self.assertRaises(ValueError):stationary_proof(self.states()[:1])
        s=self.states();s[-1]['poses']['odom'][0]+=.03
        with self.assertRaises(ValueError):stationary_proof(s)
    def test_duplicate_feedback_is_not_proof(self):
        s=self.states();s[1]['stamp_ns']=s[0]['stamp_ns']
        with self.assertRaises(ValueError):stationary_proof(s)

    def test_stop_confirmation_does_not_require_camera_synchronization(self):
        import tempfile
        from pathlib import Path
        from motion_stop import StopController
        states=iter(self.states())
        def no_camera(**kwargs):raise ValueError('action_measurement_unsynchronised_or_clock_changed')
        runtime=SimpleNamespace(sample=no_camera,pump=lambda dt:None,state=lambda:next(states))
        with tempfile.TemporaryDirectory() as directory:
            controller=StopController(runtime,SimpleNamespace(),MotionGate(),Path(directory))
            try:self.assertTrue(controller._verify({'stage':'observe_target'},False)['motion_stopped'])
            finally:controller.close()

    def test_fast_feedback_accumulates_full_interval(self):
        import tempfile
        from pathlib import Path
        from motion_stop import StopController
        counter=[0]
        def state():
            counter[0]+=50_000_000
            return dict(stamp_ns=counter[0],joints=[0,0,0,0,0,.3],poses={'odom':[1,2,0]})
        runtime=SimpleNamespace(pump=lambda dt:None,state=state)
        with tempfile.TemporaryDirectory() as directory:
            controller=StopController(runtime,SimpleNamespace(),MotionGate(),Path(directory))
            try:
                proof=controller._verify({'stage':'observe_target'},False)
                self.assertGreaterEqual(proof['feedback_stamps_ns'][-1]-proof['feedback_stamps_ns'][0],300_000_000)
            finally:controller.close()

    def test_carry_navigation_failure_keeps_actual_report_path(self):
        from pathlib import Path
        runtime=SimpleNamespace(publisher=lambda *a:None)
        def fail(*a,**kw):raise RuntimeError('child_failed')
        motion=SimpleNamespace(command=fail);gate=MotionGate();gate.install(runtime,motion)
        with self.assertRaises(RuntimeError):motion.command(['/code/carry_navigation_runtime.py','--output','/tmp/owned-carry.json'])
        self.assertTrue(gate.child_command_uncertain)
        self.assertEqual(gate.navigation_report_path,Path('/tmp/owned-carry.json'))
        with self.assertRaises(RuntimeError):motion.command(['/code/outcome_finite_adjustment.py','--output','/tmp/adjust.json'])
        self.assertIsNone(gate.navigation_report_path)

    def test_python_prefixed_navigation_retains_report(self):
        from pathlib import Path
        runtime=SimpleNamespace(publisher=lambda *a:None)
        def fail(*a,**kw):raise RuntimeError('failed')
        motion=SimpleNamespace(command=fail);g=MotionGate();g.install(runtime,motion)
        with self.assertRaises(RuntimeError):
            motion.command(['/usr/bin/python3','/code/carry_navigation_runtime.py','--output','/tmp/nav.json'])
        self.assertEqual(g.navigation_report_path,Path('/tmp/nav.json'))

    def test_navigation_failure_inside_grasp_or_withdraw_can_retire(self):
        import tempfile,json
        from pathlib import Path
        from motion_stop import StopController
        for stage in ('grasp','withdraw'):
            with self.subTest(stage=stage),tempfile.TemporaryDirectory() as d:
                g=MotionGate();g.child_command_uncertain=True;g.navigation_report_path=Path(d)/'nav.json'
                g.navigation_report_path.write_text(json.dumps(dict(status='failed',nav_reset=True,carry_footprint_restored=True)))
                states=iter(self.states());rt=SimpleNamespace(pump=lambda dt:None,state=lambda:next(states))
                controller=StopController(rt,SimpleNamespace(active_child=None),g,Path(d))
                try:self.assertTrue(controller._verify(dict(stage=stage,operation_id='r/5'),False)['motion_stopped'])
                finally:controller.close()

    def test_nav_failure_with_no_reset_or_live_child_is_not_retired(self):
        import tempfile,json
        from pathlib import Path
        from motion_stop import StopController
        with tempfile.TemporaryDirectory() as d:
            g=MotionGate();g.child_command_uncertain=True;g.navigation_report_path=Path(d)/'nav.json'
            g.navigation_report_path.write_text(json.dumps(dict(nav_reset=False)))
            m=SimpleNamespace(active_child=None);c=StopController(SimpleNamespace(),m,g,Path(d))
            try:
                with self.assertRaisesRegex(RuntimeError,'navigation_stop_not_confirmed'):c._verify(dict(stage='grasp'),False)
                g.navigation_report_path.write_text(json.dumps(dict(nav_reset=True)))
                m.active_child=SimpleNamespace(poll=lambda:None)
                with self.assertRaisesRegex(RuntimeError,'navigation_child_still_running'):c._verify(dict(stage='grasp'),False)
            finally:c.close()

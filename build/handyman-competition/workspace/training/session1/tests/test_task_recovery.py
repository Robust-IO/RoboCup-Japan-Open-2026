import unittest
from threading import Event
from unittest.mock import patch
from pathlib import Path
from task_recovery import retry_read

class RecoveryTests(unittest.TestCase):
    def test_fresh_measurement_retries_then_returns_real_result(self):
        calls=[];logs=[]
        def read():
            calls.append(1)
            if len(calls)<3:raise ValueError('new_destination_depth_frame_required')
            return {'stamp':123}
        with patch('task_recovery.time.sleep'):
            self.assertEqual(retry_read(read,record=logs.append),{'stamp':123})
        self.assertEqual(len(logs),2)
    def test_cancellation_interrupts_recovery_without_new_motion(self):
        cancel=Event()
        def read():cancel.set();raise ValueError('stale_camera')
        with self.assertRaisesRegex(RuntimeError,'task_cancelled'):retry_read(read,cancel=cancel)
    def test_programming_error_not_converted_to_success(self):
        def read():raise ValueError('wrong_grasp_context')
        with self.assertRaisesRegex(ValueError,'wrong_grasp_context'):retry_read(read)
    def test_navigation_has_no_12_second_translation_abort(self):
        import ast
        path=Path(__file__).resolve().parents[1]/'carry_navigation_runtime.py'
        tree=ast.parse(path.read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)]
        self.assertFalse(any(isinstance(n.func,ast.Name) and n.func.id=='ProgressWatch' for n in calls))
        self.assertNotIn('carry_no_progress_reposition_required',path.read_text())

class ContainerRecoveryTests(unittest.TestCase):
    def test_inconclusive_more_than_twelve_frames_does_not_release_twice(self):
        from test_container_transfer import Adapter,row
        from container_transfer import drop_and_check,ContainerPolicy
        adapter=Adapter();count=[0];original=adapter.capture_external_floor
        def capture():
            count[0]+=1
            if count[0]<=15:return {'valid_frame':False}
            return original()
        adapter.capture_external_floor=capture
        class Cancel:
            def is_set(self):return False
            def wait(self,t):return False
        result=drop_and_check(adapter,'canned_juice','bin#living_room',cancel=Cancel(),policy=ContainerPolicy(persistent_recovery=True))
        self.assertEqual(result['status'],'ok');self.assertEqual(adapter.releases,1);self.assertGreater(count[0],15)

class RetirementRecoveryTests(unittest.TestCase):
    def test_failed_finite_goal_can_retire_after_fresh_stationary_proof(self):
        import tempfile
        from types import SimpleNamespace
        from motion_stop import StopController,MotionGate
        stamps=iter((1,200000001,400000001))
        rt=SimpleNamespace(pump=lambda _:None,state=lambda:dict(stamp_ns=next(stamps),joints=[0]*6,poses={'odom':[0]*3}))
        gate=MotionGate();gate.child_command_uncertain=True
        with tempfile.TemporaryDirectory() as d:
            c=StopController(rt,SimpleNamespace(active_child=SimpleNamespace(poll=lambda:1)),gate,Path(d))
            try:self.assertTrue(c._verify({'stage':'grasp'},False)['motion_stopped'])
            finally:c.close()
    def test_withdrawal_continues_after_old_five_correction_budget(self):
        from types import SimpleNamespace
        from table_withdrawal import TableWithdrawal
        count=[0];moves=[]
        def observe():
            count[0]+=1
            return dict(points_map=[[1 if count[0]<9 else -2,0,0]],table_center=[0,0],table_half=[.2,.2],heading=[1,0],evidence_ref='fresh')
        rt=SimpleNamespace(state=lambda:dict(poses={'map':[0]*3},joints=[0]*6))
        motion=SimpleNamespace(adjust=lambda *a,**kw:moves.append(a),session_cancel_event=Event())
        proof=TableWithdrawal(rt,motion,observe)()
        self.assertTrue(proof['clear_to_fold']);self.assertEqual(len(moves),8)

class GraspResetRecoveryTests(unittest.TestCase):
    def test_direct_reset_observation_retries_and_restores_backend(self):
        from task_recovery import grasp_observation_recovery
        from grasp_runtime.autograsp.coordinator import AttemptFailure
        from grasp_runtime.autograsp.recovery import Failure
        from types import SimpleNamespace
        calls=[];heads=[]
        def original(target):
            calls.append(target)
            if len(calls)==1:raise AttemptFailure(Failure.OCCLUDED,'requested_identity_not_unique_in_current_image')
            return 'new_scene'
        backend=SimpleNamespace(observe_stopped=original,record=lambda row:None)
        with grasp_observation_recovery(backend,SimpleNamespace(face_source=heads.append),Event()):
            self.assertEqual(backend.observe_stopped('canned_juice'),'new_scene')
        self.assertIs(backend.observe_stopped,original);self.assertEqual(heads,[(0.,-.3)])
    def test_cancel_in_reset_does_not_leak_wrapped_observer(self):
        from task_recovery import grasp_observation_recovery
        from types import SimpleNamespace
        cancel=Event();cancel.set();original=lambda target:self.fail('must not observe')
        backend=SimpleNamespace(observe_stopped=original)
        with self.assertRaisesRegex(RuntimeError,'task_cancelled'):
            with grasp_observation_recovery(backend,SimpleNamespace(),cancel):backend.observe_stopped('can')
        self.assertIs(backend.observe_stopped,original)

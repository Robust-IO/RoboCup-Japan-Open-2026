import copy
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from apple_transfer_automation import RobotLock,write_json
from can_transfer_automation import STAGES,OfflineBackend,configuration,run
from can_transfer_context import stage_lock,build_context,load_context
from test_canned_grasp_geometry import snapshot,shape


class CoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.console=contextlib.redirect_stdout(io.StringIO());self.console.__enter__()
    def tearDown(self):self.console.__exit__(None,None,None);self.tmp.cleanup()
    def test_all_stages_offline_never_claim_physical_success(self):
        backend=OfflineBackend();result=run(configuration(),'case',self.root/'new',backend)
        self.assertEqual(backend.calls,list(STAGES));self.assertFalse(result['physical_transfer_verified'])
        self.assertFalse(result['uninterrupted_transfer_verified']);self.assertFalse(result['robot_commands_sent'])
        self.assertFalse(result['task_finished_sent']);self.assertFalse(result['historical_checkpoint_used'])
    def test_each_failure_stops_later_motion_and_never_retries(self):
        for index,stage in enumerate(STAGES):
            backend=OfflineBackend(fail=stage);result=run(configuration(),'case',self.root/stage,backend)
            self.assertEqual(backend.calls,list(STAGES[:index+1]));self.assertFalse(result['physical_transfer_verified'])
            self.assertEqual(result['status'],'stopped_offline');self.assertFalse(result['automatic_action_retry'])
    def test_completed_output_cannot_replay(self):
        run(configuration(),'case',self.root/'new',OfflineBackend());b=OfflineBackend()
        with self.assertRaises(FileExistsError):run(configuration(),'case',self.root/'new',b)
        self.assertEqual(b.calls,[])
    def test_interrupted_carry_never_starts_place(self):
        class Interrupted(OfflineBackend):
            def perform(self,stage):
                if stage=='carry_destination':raise KeyboardInterrupt()
                return super().perform(stage)
        b=Interrupted();result=run(configuration(),'case',self.root/'new',b)
        self.assertNotIn('place_verify',b.calls);self.assertEqual(result['reason'],'interrupted')
    def test_missing_or_outside_current_run_evidence_cannot_pass(self):
        path=self.root/'outside.json';write_json(path,dict(synthetic=True))
        class Outside(OfflineBackend):
            def perform(self,stage):return dict(verified=True,evidence=str(path))
        result=run(configuration(),'case',self.root/'new',Outside())
        self.assertEqual(result['confirmed_stages'],[]);self.assertIn('outside',result['reason'])
    def test_modified_route_or_navigation_policy_is_rejected(self):
        config=configuration();config['baseline']['navigation_tolerances']=[.2,.2,.2]
        with self.assertRaises(ValueError):run(config,'case',self.root/'new',OfflineBackend())
    def test_default_common_entries_do_not_import_ros_or_send_commands(self):
        for name in ('apple','canned_juice'):
            script=Path(__file__).with_name('run_object_transfer.py')
            proc=subprocess.run([sys.executable,str(script),'--object',name],capture_output=True,text=True,timeout=15)
            self.assertEqual(proc.returncode,0,proc.stderr)
            value=next(json.loads(line) for line in proc.stdout.splitlines() if line.startswith('{'))
            self.assertFalse(value['robot_commands_sent']);self.assertFalse(value['navigation_parameters_changed'])


class OwnershipTests(unittest.TestCase):
    def test_borrowed_lock_does_not_release_parent_or_allow_other_controller(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);write_json(root/'run-meta.json',dict(run_id='test'))
            lock_path=str(root/'test.lock')
            with patch('can_transfer_context.LOCK_PATH',lock_path),RobotLock(lock_path) as owner:
                with stage_lock(owner.stream.fileno(),root):
                    with self.assertRaises(RuntimeError):
                        with RobotLock(lock_path):pass
                with self.assertRaises(RuntimeError):
                    with RobotLock(lock_path):pass
            with RobotLock(lock_path):pass
    def test_independent_file_descriptor_cannot_impersonate_parent_lock(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);write_json(root/'run-meta.json',dict(run_id='test'));lock_path=str(root/'test.lock')
            with patch('can_transfer_context.LOCK_PATH',lock_path),RobotLock(lock_path):
                with open(lock_path,'a+') as other:
                    with self.assertRaises(BlockingIOError):
                        with stage_lock(other.fileno(),root):pass
    def test_real_subprocess_can_borrow_inherited_descriptor(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);write_json(root/'run-meta.json',dict(run_id='test'));lock_path=str(root/'test.lock')
            with RobotLock(lock_path) as lock:
                program='import sys,can_transfer_context as c; c.LOCK_PATH=sys.argv[1]; lease=c.stage_lock(int(sys.argv[2]),sys.argv[3]); lease.__enter__(); lease.__exit__(None,None,None)'
                proc=subprocess.run([sys.executable,'-c',program,lock_path,str(lock.stream.fileno()),str(root)],
                    pass_fds=(lock.stream.fileno(),),cwd=Path(__file__).parent,capture_output=True,text=True,timeout=15)
                self.assertEqual(proc.returncode,0,proc.stderr)
                with self.assertRaises(RuntimeError):
                    with RobotLock(lock_path):pass


class ContextTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name).resolve()
        write_json(self.root/'run-meta.json',dict(run_id='new-run',object='canned_juice',mode='live',started_ns=900_000_000,
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
        grasp=self.root/'grasp';before=grasp/'captures/preclose';before.mkdir(parents=True)
        write_json(before/'snapshot.json',snapshot());write_json(before/'can-measurement.json',shape())
        rows=[]
        for i in range(3):
            stamp=2_000_000_000+i*200_000_000;path=grasp/f'captures/after-lift-{i:02d}';path.mkdir()
            s=snapshot(stamp,lift=.3808021848314039,palm_height=.658,opening=.48)
            m=shape(center=(.6892952185,.078,.658),stamp=stamp)
            write_json(path/'snapshot.json',s);write_json(path/'can-measurement.json',m)
            (path/'rgb.png').write_bytes(b'SYNTHETIC_CONTEXT_PATH_TEST_ONLY');(path/'depth_m.npy').write_bytes(b'SYNTHETIC_NO_REAL_DEPTH')
            rows.append(dict(capture=str(path),stamp_ns=stamp))
        self.report=dict(status='can_grasp_and_lift_verified',grasp_success_verified=True,task_id='grasp',
            close=dict(status='passed',grasp_closure_commanded=True,before_capture=str(before)),lift=dict(status='passed'),
            held_opening_rad=.48,lift_verification=rows,reference=dict(height_m=.125),
            grasp_plan=dict(contact_start_lift_m=.2808021848314039))
        self.result=grasp/'result.json';write_json(self.result,self.report)
    def tearDown(self):self.tmp.cleanup()
    def test_reference_uses_this_grasp_actual_relative_offset_and_height(self):
        path=build_context(self.root);value=load_context(path,self.root/'carry')
        self.assertFalse(value['historical_checkpoint_used']);self.assertEqual(value['run_id'],'new-run')
        self.assertAlmostEqual(value['held_center_minus_palm_base_m'][0],.0621258700082378)
        self.assertAlmostEqual(value['estimated_upright_bottom_below_palm_m'],.0625)
    def test_failed_grasp_or_missing_lift_rows_cannot_create_context(self):
        for key,value in (('status','stopped_requires_actual_state_review'),('lift_verification',[])):
            report=dict(self.report,**{key:value});write_json(self.result,report)
            with self.assertRaises(ValueError):build_context(self.root)
    def test_wrong_class_or_nonfollowing_lift_cannot_create_context(self):
        path=Path(self.report['lift_verification'][0]['capture'])/'can-measurement.json';m=json.loads(path.read_text())
        m['center_m'][2]-=.10;write_json(path,m)
        with self.assertRaises(ValueError):build_context(self.root)
    def test_old_captures_are_not_reused_after_new_run_start(self):
        meta=json.loads((self.root/'run-meta.json').read_text());meta['started_ns']=3_000_000_000
        write_json(self.root/'run-meta.json',meta)
        with self.assertRaises(ValueError):build_context(self.root)
    def test_evidence_tampering_cross_run_or_new_boot_is_rejected(self):
        path=build_context(self.root);value=json.loads(path.read_text())
        with self.assertRaises(ValueError):load_context(path,self.root/'other-run/carry')
        write_json(path,dict(value,boot_id='other-boot'))
        with self.assertRaises(ValueError):load_context(path,self.root/'carry')
        write_json(path,value);source=Path(value['source_capture']);(source/'rgb.png').write_bytes(b'changed')
        with self.assertRaises(ValueError):load_context(path,self.root/'carry')
    def test_placement_requires_this_run_successful_carry(self):
        path=build_context(self.root);carry=self.root/'carry';carry.mkdir()
        proof=dict(status='can_retraction_and_destination_navigation_verified',retraction_verified=True,
            destination_navigation_verified=True,context_run_id='new-run',hand_released=False)
        write_json(carry/'result.json',dict(proof,context_run_id='previous-run'))
        with self.assertRaises(ValueError):load_context(path,self.root/'place',require_carried=True)
        write_json(carry/'result.json',proof)
        self.assertEqual(load_context(path,self.root/'place',require_carried=True)['run_id'],'new-run')
    def test_recorded_replay_context_is_never_a_live_handoff(self):
        path=build_context(self.root);meta=json.loads((self.root/'run-meta.json').read_text());meta['mode']='recorded_replay'
        write_json(self.root/'run-meta.json',meta)
        with self.assertRaises(ValueError):load_context(path,self.root/'carry')


if __name__=='__main__':unittest.main()

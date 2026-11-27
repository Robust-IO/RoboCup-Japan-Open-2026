import json
import tempfile
import unittest
from pathlib import Path
from threading import Event
from types import SimpleNamespace
from unittest.mock import patch
from transfer_bindings import TransferBindings
class TransferBindingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.effect=dict(session_id='s',run_id='r',operation_id='r/7');self.cancel=Event()
        context=dict(target='dog_doll',source_table='white_side_table#living_room',schema='handyman_held_surface_v1',frame='hand_palm_link',vertices_palm_m=[[0,0,-.05],[.05,.05,.03]])
        runtime=SimpleNamespace(face_source=lambda *a:None,fresh_measurement=lambda s:None)
        backend=SimpleNamespace(_sample=lambda:dict(snapshot={'streams':{'depth':{'stamp_ns':50}},'joints':{'hand_motor_joint':.65}}),worker=object())
        self.stages=SimpleNamespace(check=lambda c:None,context=context,runtime=runtime,grasp_backend=backend,
            config={},environment={},output=Path(self.tmp.name),motion=object())
        observer=lambda *a,**kw:SimpleNamespace(measure_and_plan=lambda:None,verify_support=lambda p:None)
        capture=lambda *a,**kw:SimpleNamespace(verify=lambda p,release_stamp_ns:dict(status='ok',placement_verified=True,stamp=release_stamp_ns))
        self.b=TransferBindings(self.stages,{'id':'table'},[[1,0,0],[0,1,0],[0,0,1]],core=object(),policy=SimpleNamespace(hand_is_open=lambda x:x>=.6),
            withdraw_source=lambda:None,withdraw_empty=lambda:None,bottom_evidence=lambda c:True,
            hold_factory=lambda *a:lambda label:None,observer_factory=observer,
            standoff_factory=lambda *a:None,approach_factory=lambda *a:None,capture_factory=capture)
    def fake_place(self,*a,**kw):return dict(status='ok',release_sent=True,cleanup_completed=True,plan={'destination_id':'table'},release_record=kw['on_release']())
    def test_release_ack_passed_to_independent_verification(self):
        with patch('transfer_bindings.place',side_effect=self.fake_place):self.b.place(self.effect,self.cancel)
        self.assertEqual(json.loads((self.stages.output/'release-record.json').read_text())['release_stamp_ns'],50)
        result=self.b.verify_placement(dict(self.effect,operation_id='r/8'),self.cancel)
        self.assertTrue(result['completion_actions_confirmed']);self.assertFalse(result['placement_verified'])
    def test_place_not_replayed(self):
        with patch('transfer_bindings.place',side_effect=self.fake_place):self.b.place(self.effect,self.cancel)
        with self.assertRaises(ValueError):self.b.place(self.effect,self.cancel)
    def test_no_release_no_verification(self):
        with self.assertRaises(ValueError):self.b.verify_placement(self.effect,self.cancel)
    def test_no_visual_stability_capture_after_cleanup(self):
        self.stages.context['target']='apple';self.stages.config={'task':{'object':'apple'}}
        self.b.capture_factory=lambda *a,**kw:self.fail('visual stability check must not run')
        with patch('transfer_bindings.place',side_effect=self.fake_place):self.b.place(self.effect,self.cancel)
        result=self.b.verify_placement(self.effect,self.cancel)
        self.assertEqual(result['target'],'apple');self.assertFalse(result['object_stability_checked'])
    def test_release_reports_without_cleanup(self):
        with patch('transfer_bindings.place',side_effect=self.fake_place):self.b.place(self.effect,self.cancel)
        self.b.placement['cleanup_completed']=False
        result=self.b.verify_placement(self.effect,self.cancel)
        self.assertEqual(result['basis'],'gripper_release_completed')
        self.assertFalse(result['cleanup_completed'])
    def test_new_run_cannot_use_old_grasp(self):
        self.b.initialize(self.effect,self.cancel)
        with self.assertRaises(ValueError):self.b.initialize(dict(self.effect,run_id='new'),self.cancel)

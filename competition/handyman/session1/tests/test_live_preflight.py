import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import unittest
import numpy as np
from live_preflight import run
class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name);self.commands=[]
        self.runtime=SimpleNamespace(publisher=lambda *a:self.commands.append('publisher'),sample=lambda **kw:None,
            state=lambda:dict(joints=[0,0,0,0,0,.65],poses={'map':[1,1,0]}),fresh_measurement=lambda s:None,
            save=lambda *a:self.path/'capture',pump=lambda t:None)
        self.motion=SimpleNamespace(command=lambda *a:self.commands.append('command'),room_at=lambda pose:'living_room')
        self.bundle=dict(runtime=self.runtime,motion_backend=self.motion,output=self.path,
            grasp_backend=SimpleNamespace(_sample=lambda:dict(snapshot={'streams':{'depth':{'stamp_ns':123}}},rgb=np.zeros((2,3,3)),depth=np.ones((2,3)))),
            worker=SimpleNamespace(infer=lambda *a,**kw:{'detections':[]}))
    def test_no_target_required_for_sensor_preflight(self):
        with patch.dict('sys.modules',{'task_outcome_policy':SimpleNamespace(hand_is_open=lambda v:v>=.6)}):r=run(self.bundle)
        self.assertEqual(r['status'],'passed');self.assertFalse(r['target_visible']);self.assertFalse(self.commands)
        self.assertTrue((self.path/'preflight-result.json').exists())
    def test_motion_attempt_is_blocked_and_reported(self):
        self.bundle['grasp_backend']._sample=lambda:self.runtime.publisher('test')
        with patch.dict('sys.modules',{'task_outcome_policy':SimpleNamespace(hand_is_open=lambda v:v>=.6)}):
            with self.assertRaises(RuntimeError):run(self.bundle)
        self.assertFalse(self.commands)
        self.assertEqual(json.loads((self.path/'preflight-result.json').read_text())['status'],'failed')
        self.runtime.publisher('restored');self.assertEqual(self.commands,['publisher'])

    def test_closed_hand_does_not_block_camera_diagnostics(self):
        self.runtime.state=lambda:dict(joints=[0,0,0,0,0,0],poses={'map':[1,1,0]})
        with patch.dict('sys.modules',{'task_outcome_policy':SimpleNamespace(hand_is_open=lambda v:v>=.6)}):r=run(self.bundle)
        self.assertEqual(r['status'],'passed');self.assertFalse(r['task_start_ready']);self.assertFalse(self.commands)

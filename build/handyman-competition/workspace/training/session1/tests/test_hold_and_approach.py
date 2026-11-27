import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from hold_observer import HoldObserver
from placement_approach import PlacementApproach
class Runtime:
    def __init__(self,root):self.root=root;self.moves=[]
    def palm_pose(self,s):return dict(translation=[0,0,0],quaternion=[0,0,0,1])
    def fresh_measurement(self,s):pass
    def save(self,item,label):p=self.root/label;p.mkdir();return p
    def state(self):return dict(joints=[.3,0,0,0,0,.3],poses={'map':[0,0,0]})
    def move_joints(self,g,l):self.moves.append((g,l))
class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.rt=Runtime(Path(self.temp.name))
    def hold(self):
        x,y=np.meshgrid(np.arange(10)*.004,np.arange(10)*.004)
        points=np.c_[x.ravel(),y.ravel(),np.full(x.size,.1)];rgb=np.full_like(points,120)
        item={'snapshot':{'streams':{'depth':{'stamp_ns':2}},'joints':{'hand_motor_joint':.3}}}
        backend=SimpleNamespace(rt=self.rt,_sample=lambda:item,_cloud=lambda i:(points,rgb,np.arange(len(points))))
        context=dict(frame='hand_palm_link',vertices_palm_m=points.tolist(),surface_rgb=rgb.tolist(),depth_stamp_ns=1,holding_opening_rad=.3)
        return HoldObserver(backend,context,rotation=lambda q:np.eye(3)),backend
    def test_current_correspondence(self):
        observer,b=self.hold();self.assertTrue(observer('test')['holding_verified'])
        with self.assertRaises(ValueError):observer('repeat')
    def test_grip_alone_not_proof(self):
        observer,b=self.hold();b._cloud=lambda i:(np.empty((0,3)),np.empty((0,3)),np.array([],int))
        with self.assertRaises(ValueError):observer('absent')
    def test_verified_lift_can_continue_without_visual_match(self):
        from handover import delivery_evidence
        observer,b=self.hold();b.record=lambda e:None
        observer.context.update(require_visual_carry_checks=False,lift_evidence={'basis':'lift_and_reclose_joint_feedback'})
        b._cloud=lambda i:self.fail('visual check must not block verified grip')
        proof=observer('after-tuck')
        self.assertTrue(proof['holding_verified'])
        self.assertTrue(delivery_evidence([0,0,1],[0,0,1],proof,target='dog_doll')['delivery_ready'])
        self.assertFalse(delivery_evidence([1,0,1],[0,0,1],proof,target='dog_doll')['delivery_ready'])
    def setup_approach(self,checks):
        calls=[]
        obs=SimpleNamespace(runtime=self.rt,config={'policy':{'release_gap_m':.015}},rotation=lambda q:np.eye(3),
            last_row={'item':{'snapshot':{'tracking_transforms':{'map_from_base_at_depth':{'quaternion':[0,0,0,1]}}}}},
            verify_support=lambda plan:next(checks))
        motion=SimpleNamespace(adjust=lambda *a,**kw:calls.append((a,kw)))
        return PlacementApproach(obs,motion,policy=SimpleNamespace(RELEASE_BOTTOM_GAP_M=(-.01,.05))),calls
    def test_full_translation_then_stop(self):
        a,calls=self.setup_approach(iter([dict(ready=False,bottom_gap_m=.015,center_map_m=[0,0,.5]),dict(ready=True)]))
        a({'release_xy_map_m':[.4,.1]})
        self.assertEqual(calls[0][0][2],[.4,.1,0.]);self.assertEqual(len(calls),1)
    def test_remeasure_after_six_corrections_instead_of_abort(self):
        a,calls=self.setup_approach(iter([dict(ready=False,bottom_gap_m=.015,center_map_m=[0,0,.5])]*7+[dict(ready=True)]))
        replans=[]
        a.observer.measure_and_plan=lambda:replans.append(True) or {'release_xy_map_m':[.3,.1]}
        self.assertTrue(a({'release_xy_map_m':[.4,.1]})['ready'])
        self.assertEqual(len(calls),6);self.assertEqual(replans,[True])
    def test_height_correction_not_base_motion(self):
        a,calls=self.setup_approach(iter([dict(ready=False,bottom_gap_m=.1),dict(ready=True)]))
        a({'release_xy_map_m':[.4,.1]});self.assertFalse(calls)
        self.assertAlmostEqual(self.rt.moves[0][0][0],.215)

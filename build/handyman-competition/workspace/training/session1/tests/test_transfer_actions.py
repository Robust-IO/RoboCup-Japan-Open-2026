import unittest
import importlib.util
from pathlib import Path
from transfer_actions import tuck,place,measured_release_shape
p=Path(__file__).resolve().parents[2]/'handyman_common_placement_v1/frozen/table_transfer_core.py'
if not p.exists():p=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/table_transfer_core.py')
spec=importlib.util.spec_from_file_location('common_core_actions',p)
core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)
class Runtime:
    def __init__(self):self.joints=[.3,-1.7,0,.1292,0,.31];self.calls=[]
    def state(self):return {'joints':list(self.joints)}
    def record(self,e):pass
    def move_joints(self,goal,label,**kw):
        if kw.get('before_publish'):kw['before_publish']()
        self.joints=list(goal);self.calls.append((label,list(goal)))
def hold(label):return {'holding_verified':True,'evidence_ref':'synthetic/'+label}
def clear():return {'clear_to_fold':True}
class ActionTests(unittest.TestCase):
    def context(self):return dict(schema='handyman_held_surface_v1',frame='hand_palm_link',target='dog_doll',vertices_palm_m=[[-.05,-.03,-.08],[.05,.03,.02]])
    def args(self):return dict(context=self.context(),release_rotation=[[1,0,0],[0,1,0],[0,0,1]],observe_hold=hold,
            measure_and_plan=lambda shape:{'palm_height_m':core.release_height(.4,shape['bottom_below_palm_m'])},
            prepare_standoff=lambda p:p,approach=lambda p:None,verify_support=lambda p:{'ready':True},withdraw_empty=clear,bottom_observed=True)
    def test_tuck_preserves_grip(self):
        rt=Runtime();r=tuck(rt,observe_hold=hold,withdraw_from_source=clear)
        self.assertTrue(r['carry_ready']);self.assertEqual([q[5] for _,q in rt.calls],[.31])
        self.assertEqual(rt.calls[0][1][:5],[0.,0.,0.,-__import__('math').pi/2,0.])
    def test_withdrawal_failure_never_folds(self):
        rt=Runtime()
        with self.assertRaises(ValueError):tuck(rt,observe_hold=hold,withdraw_from_source=lambda:{})
        self.assertFalse(rt.calls)
    def test_place_reuses_height_and_opens_once(self):
        rt=Runtime();r=place(rt,core,**self.args())
        self.assertEqual(sum(label=='release_once' for label,_ in rt.calls),1)
        self.assertTrue(r['release_sent']);self.assertFalse(r['placement_verified']);self.assertTrue(r['cleanup_completed'])
        self.assertAlmostEqual(r['plan']['palm_height_m'],.495)
    def test_support_failure_does_not_open(self):
        rt=Runtime();a=self.args();a['verify_support']=lambda p:{'ready':False}
        with self.assertRaises(ValueError):place(rt,core,**a)
        self.assertNotIn('release_once',[label for label,_ in rt.calls])
    def test_cleanup_failure_retains_release_result(self):
        rt=Runtime();a=self.args();a['withdraw_empty']=lambda:{}
        r=place(rt,core,**a);self.assertTrue(r['release_sent']);self.assertFalse(r['cleanup_completed'])
    def test_shape_rotated_before_height(self):
        r=measured_release_shape(self.context(),[[0,0,1],[0,1,0],[-1,0,0]])
        self.assertAlmostEqual(r['bottom_below_palm_m'],.05)

    def test_occluded_before_release_does_not_block_geometry_release(self):
        rt=Runtime();a=self.args()
        def observe(label):
            if label=='before-release':raise ValueError('occluded')
            return hold(label)
        a['observe_hold']=observe
        a['verify_release']=lambda p:{'ready':True,'gripper_center_inside_destination':True}
        self.assertTrue(place(rt,core,**a)['release_sent'])
    def test_prepared_plan_skips_approach_and_posture(self):
        rt=Runtime();a=self.args()
        def forbidden(*args):raise AssertionError('completed action replayed')
        a.update(prepared_plan={'palm_height_m':.5},measure_and_plan=forbidden,prepare_standoff=forbidden,approach=forbidden,observe_hold=forbidden)
        r=place(rt,core,**a)
        self.assertTrue(r['release_sent'])
        self.assertEqual(rt.calls[0][0],'release_once')

import importlib.util
from pathlib import Path
import unittest
import numpy as np
from placement_verification import verify_placement,support_check
root=Path(__file__).resolve().parents[2]/'handyman_common_placement_v1/frozen'
if not root.exists():root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
def module(name):
    spec=importlib.util.spec_from_file_location(name,root/(name+'.py'))
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
support_module=module('tabletop_support');policy=module('task_outcome_policy')
class VerificationTests(unittest.TestCase):
    def setUp(self):
        x,y=np.meshgrid(np.arange(-.5,.5,.004),np.arange(-.5,.5,.004))
        points=np.c_[x.ravel(),y.ravel(),np.full(x.size,.4)]
        model=support_module.build_support(points,[0,0],[.5,.5],[0,0,.4],'table',1)
        self.support=support_module.TableSupport(model,'table')
        self.plan=dict(destination_id='table',plane_map=[0,0,.4],radius_m=.06,release_xy_map_m=[-.2,-.2],footprint_center=[0,0],footprint_half_size=[.5,.5])
        self.rows=[]
        for stamp in [200_000_000,400_000_000,600_000_000]:
            self.rows.append(dict(depth_stamp_ns=stamp,target='dog_doll',destination_id='table',geometry_source='independent_target_RGBD',bottom_observed=True,evidence_ref='synthetic/'+str(stamp),points_map=[[-.04,-.02,.405],[.04,.02,.46]],plane=dict(plane_map=[0,0,.4],destination_id='table',source_depth_stamp_ns=stamp)))
    def verify(self):return verify_placement(self.rows,self.plan,self.support,policy,target='dog_doll',release_stamp_ns=100_000_000,joints=[0,0,0,0,0,.65])
    def test_supported_not_exact_planned_position(self):
        self.assertTrue(self.verify()['placement_verified'])
    def test_held_estimate_cannot_verify_release(self):
        self.rows[0]['geometry_source']='held_kinematic'
        with self.assertRaises(ValueError):self.verify()
    def test_old_or_repeated_frames_rejected(self):
        self.rows[1]=dict(self.rows[0])
        with self.assertRaises(ValueError):self.verify()
    def test_wrong_target_rejected(self):
        self.rows[0]['target']='apple'
        with self.assertRaises(ValueError):self.verify()
    def test_hanging_above_table_rejected(self):
        self.rows[0]['points_map']=[[-.04,-.02,.5],[.04,.02,.56]]
        with self.assertRaises(ValueError):self.verify()
    def test_stable_near_edge_accepted(self):
        for row in self.rows:row['points_map']=[[.46,0,.405],[.49,.02,.46]]
        self.assertTrue(self.verify()['placement_verified'])
    def test_postrelease_has_no_extra_center_containment_gate(self):
        for row in self.rows:row['points_map']=[[.56,0,.405],[.59,.02,.46]]
        self.assertTrue(self.verify()['placement_verified'])
    def test_tight_disk_is_diagnostic_after_release_but_required_before(self):
        self.plan.update(placement_selection_basis='tight_object_disk',
            support_footprint={'center':{'x':0,'y':0},'size':{'x':1,'y':1}},object_disk_edge_gap_m=.005)
        for row in self.rows:row['points_map']=[[.46,0,.405],[.49,.02,.46]]
        result=self.verify()
        self.assertTrue(result['placement_verified'])
        self.assertFalse(result['checks'][0]['edge']['safe'])
        self.assertTrue(result['checks'][0]['edge_is_diagnostic_only'])
        self.assertFalse(support_check(self.rows[0]['points_map'],self.rows[0]['plane'],self.plan,self.support,policy)['ready'])
    def test_moving_object_rejected(self):
        self.rows[-1]['points_map']=[[.2,0,.405],[.28,.02,.46]]
        with self.assertRaises(ValueError):self.verify()
    def test_prerelease_uses_same_support_module(self):
        self.assertTrue(support_check(self.rows[0]['points_map'],self.rows[0]['plane'],self.plan,self.support,policy)['ready'])

    def test_previous_plane_height_is_not_a_release_gate(self):
        self.plan['plane_map']=[0,0,.7]
        row=self.rows[0]
        result=support_check(row['points_map'],row['plane'],self.plan,self.support,policy)
        self.assertAlmostEqual(result['bottom_gap_m'],.005)
        wrong=dict(row['plane'],destination_id='other_table')
        with self.assertRaisesRegex(ValueError,'wrong_destination_plane'):
            support_check(row['points_map'],wrong,self.plan,self.support,policy)

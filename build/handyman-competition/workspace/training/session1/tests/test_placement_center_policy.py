import unittest
from types import SimpleNamespace
import numpy as np
from placement_center_policy import observed_centers,free_space_clearance,free_space_rank
class CenterPolicyTests(unittest.TestCase):
    def test_larger_space_beats_shorter_travel(self):
        self.assertLess(free_space_rank(.15,1.,[0,0]),free_space_rank(.05,.1,[0,0]))
    def test_similar_space_prefers_shorter_travel(self):
        self.assertLess(free_space_rank(.141,.1,[0,0]),free_space_rank(.159,1.,[0,0]))
    def test_obstacle_and_unknown_bound_clearance(self):
        s=SimpleNamespace(mask=np.ones((40,40),bool),origin=np.array([0.,0.]))
        clear=free_space_clearance(s,np.empty((0,3)),[0,0,.4])
        blocked=free_space_clearance(s,np.array([[.1025,.1025,.5]]),[0,0,.4])
        self.assertGreater(clear[20,20],clear[0,0])
        self.assertEqual(blocked[20,20],0)
        self.assertLess(blocked[20,21],clear[20,21])
        s.mask[:,20:]=False
        unknown=free_space_clearance(s,np.empty((0,3)),[0,0,.4])
        self.assertEqual(unknown[20,25],0)
    def test_retains_observed_cells_without_full_object_disk(self):
        s=SimpleNamespace(mask=np.eye(40,dtype=bool),origin=np.array([-.1,-.1]))
        p=observed_centers(s,[0,0],[.1,.1])
        self.assertGreater(len(p),0)
        self.assertTrue((np.abs(p)<=.08).all())
    def test_unknown_cells_not_invented(self):
        s=SimpleNamespace(mask=np.zeros((40,40),bool),origin=np.array([-.1,-.1]))
        self.assertEqual(len(observed_centers(s,[0,0],[.1,.1])),0)
    def test_invalid_inset_rejected(self):
        with self.assertRaises(ValueError):observed_centers(None,[0,0],[.1,.1],0)
    def test_center_plan_is_not_rejected_by_old_full_disk_gate(self):
        from placement_verification import support_check
        p={'destination_id':'table','plane_map':[0,0,.5],'radius_m':.15,
           'footprint_center':[0,0],'footprint_half_size':[.5,.5],
           'placement_selection_basis':'observed_center_local_patch'}
        plane={'destination_id':'table','plane_map':[0,0,.5]}
        support=SimpleNamespace(query=lambda xy,r:{'safe':False})
        policy=SimpleNamespace(RELEASE_BOTTOM_GAP_M=(0,.05))
        self.assertTrue(support_check([[.4,0,.52],[.45,.01,.55]],plane,p,support,policy)['ready'])
        self.assertFalse(support_check([[.6,0,.52],[.65,.01,.55]],plane,p,support,policy)['ready'])
        self.assertFalse(support_check([[.4,0,.7],[.45,.01,.75]],plane,p,support,policy)['ready'])

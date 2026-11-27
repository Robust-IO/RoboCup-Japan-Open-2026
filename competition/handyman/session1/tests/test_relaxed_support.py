import unittest
import numpy as np
from session_table_support import build_support, TableSupport, support_from_plan
from session_placement_policy import relaxed_support_policy

class RelaxedSupportTests(unittest.TestCase):
    def test_more_candidates_without_claiming_unknown_supported(self):
        x,y=np.meshgrid(np.arange(-.3,.3,.004),np.arange(-.3,.3,.004))
        points=np.c_[x.ravel(),y.ravel(),np.full(x.size,.4)]
        model=build_support(points,[0,0],[.3,.3],[0,0,.4],'table',1)
        old=TableSupport(model,'table',edge_gap_m=.02,uncertainty_m=.01)
        new=TableSupport(model,'table',edge_gap_m=.01,uncertainty_m=.005)
        self.assertGreater(len(new.safe_centers(.1,0)),len(old.safe_centers(.1,.005)))
        self.assertFalse(new.query([.29,0],.1)['safe'])
        self.assertFalse(new.query([.5,0],.1)['safe'])
        plan=dict(destination_id='table',source_depth_stamp_ns=1,observed_table_support=model,
                  table_edge_policy=dict(version=2,profile='outcome_v1',minimum_outer_gap_m=.01,uncertainty_allowance_m=.005))
        restored=support_from_plan(plan)
        self.assertEqual(restored.query([0,0],.1),new.query([0,0],.1))
    def test_shared_policy_unchanged(self):
        original={'placement':{'outer_edge_gap_m':.02,'minimum_obstacle_points':8}}
        new=relaxed_support_policy(original)
        self.assertEqual(original['placement']['outer_edge_gap_m'],.02)
        self.assertEqual(new['placement']['minimum_obstacle_points'],8)
        self.assertEqual(new['placement']['planning_reserve_m'],0.)

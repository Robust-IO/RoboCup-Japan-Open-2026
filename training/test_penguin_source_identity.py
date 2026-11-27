import copy,unittest
from test_penguin_geometry import rendered_penguin
from penguin_geometry import estimate_source
from penguin_source_identity import make_seed,proposal,confirm,source_table_matches

class PenguinSourceIdentityTests(unittest.TestCase):
    def test_supported_overhang_can_match_source_but_disjoint_object_cannot(self):
        self.assertTrue(source_table_matches([10.6375,-6.1767,.53],[11.00287,-6.409],[.360,.780],.049))
        self.assertFalse(source_table_matches([10.58,-6.17,.53],[11.00287,-6.409],[.360,.780],.049))
        self.assertFalse(source_table_matches([1.04,1.04,.53],[0.,0.],[1.,1.],.049))
    def setUp(self):
        self.rgb,self.depth,self.s,self.d,_,_=rendered_penguin()
        self.s['tracking_epoch']='live-source'
        self.s['tracking_transforms']=dict(odom_from_base_at_depth=dict(
            translation=[0,0,0],quaternion=[0,0,0,1],stamp_ns=1_000_000_000,
            source_frame='base_link',target_frame='odom'))
        self.ref=estimate_source(self.rgb,self.depth,self.s,self.d)
        self.seed=make_seed(self.ref,self.s,self.d,'capture-original')
        self.new=copy.deepcopy(self.s)
        for name in ('rgb','depth'):self.new['streams'][name]['stamp_ns']=2_000_000_000
        self.new['tracking_transforms']['odom_from_base_at_depth']['stamp_ns']=2_000_000_000
    def test_high_confidence_seed_does_not_claim_new_classifier_detection(self):
        d=proposal(self.seed,self.new);self.assertIsNotNone(d)
        self.assertFalse(d['fresh_model_detection']);self.assertEqual(d['confidence'],.94)
        m=estimate_source(self.rgb,self.depth,self.new,d)
        proof=confirm(self.seed,m,self.new)
        self.assertFalse(proof['current_model_detection_used']);self.assertFalse(proof['new_identity_created'])
        self.assertEqual(proof['current_depth_stamp_ns'],2_000_000_000)
        self.assertFalse(self.seed['motion_authorized'])
    def test_seed_needs_high_confidence_same_class_and_new_epoch_consistent_rgbd(self):
        for d in (dict(self.d,confidence=.7),dict(self.d,name='ground_pepper')):
            with self.assertRaises(ValueError):make_seed(self.ref,self.s,d,'bad')
        with self.assertRaises(ValueError):proposal(self.seed,self.s)
        self.new['tracking_epoch']='after-unity-restart'
        with self.assertRaises(ValueError):proposal(self.seed,self.new)
    def test_different_current_object_position_cannot_inherit_the_identity(self):
        m=estimate_source(self.rgb,self.depth,self.new,self.d)
        m['cap_center_m'][0]+=.04
        with self.assertRaises(ValueError):confirm(self.seed,m,self.new)

if __name__=='__main__':unittest.main()

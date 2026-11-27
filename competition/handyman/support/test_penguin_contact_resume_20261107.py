import copy,unittest
import numpy as np
from scipy.spatial.transform import Rotation
from penguin_contact_resume_20261107 import complete_recorded_hand_inverse,validate_contact
from profiled_capture import transform_requests

class ContactTests(unittest.TestCase):
    def fixture(self):
        return dict(transform_basis='stationary_transform_snapshot',stop_observe_act=dict(captured_after_stop=True),
            streams=dict(hand=dict(stamp_ns=1_020_000_000),depth=dict(stamp_ns=1_000_000_000)),tracking_transforms=dict(
                odom_from_base_at_depth=dict(translation=[2.,-3.,.1],quaternion=Rotation.from_euler('z',.4).as_quat().tolist(),
                    stamp_ns=1_000_000_000,source_frame='base_link',target_frame='odom')))
    def test_inverse_roundtrip_and_original_capture_unchanged(self):
        s=self.fixture();before=copy.deepcopy(s);fixed=complete_recorded_hand_inverse(s)
        a=s['tracking_transforms']['odom_from_base_at_depth'];b=fixed['tracking_transforms']['base_from_odom_at_hand']
        point=np.array([.5,.1,.2]);world=Rotation.from_quat(a['quaternion']).apply(point)+a['translation']
        np.testing.assert_allclose(Rotation.from_quat(b['quaternion']).apply(world)+b['translation'],point,atol=1e-12)
        self.assertEqual(s,before);self.assertEqual(b['stamp_ns'],s['streams']['hand']['stamp_ns'])
    def test_no_stationary_proof_or_unsynchronized_capture_is_rejected(self):
        s=self.fixture();s['stop_observe_act']['captured_after_stop']=False
        with self.assertRaises(ValueError):complete_recorded_hand_inverse(s)
        s=self.fixture();s['streams']['hand']['stamp_ns']+=200_000_000
        with self.assertRaises(ValueError):complete_recorded_hand_inverse(s)
    def test_future_paired_capture_requests_real_inverse_at_hand_stamp(self):
        selected=dict(rgb=10,depth=10,hand=20)
        class M:header=type('H',(),dict(frame_id='camera'))()
        class R:history={k:{v:M()} for k,v in selected.items()}
        requests=transform_requests(R(),'paired',selected)
        self.assertIn(('base_from_odom_at_hand','base_link','odom',20),requests)
    def test_contact_validation_prevents_replaying_a_changed_grasp_or_boot(self):
        state=dict(joints=[.322,-1.7,0.,.1292,0.,.227],poses=dict(odom=[8.,-6.,0.]))
        previous=dict(status='stopped_requires_actual_state_review',class_name='toy_penguin',stage='close_once',
            reason="'base_from_odom_at_hand'",close_commands_sent=1,release_commands_sent=0,close=dict(status='passed'),last_fresh_robot_state=state)
        validate_contact(previous,dict(boot_id='same'),state,'same')
        moved=copy.deepcopy(state);moved['poses']['odom'][0]+=.1
        with self.assertRaises(ValueError):validate_contact(previous,dict(boot_id='same'),moved,'same')
        with self.assertRaises(ValueError):validate_contact(previous,dict(boot_id='old'),state,'same')

if __name__=='__main__':unittest.main()

import copy
import json
import unittest
from unittest.mock import patch

import numpy as np

from audit_preclose_identity_lift_follow import associate,audit
from verify_apple_lift_sequence import pair


def scene(centers=((0,0,.6),),radius=.04,camera_z=0):
    n=120;v,u=np.indices((n,n));ray=np.stack(((u-60)/200,(v-60)/200,np.ones((n,n))),-1)
    a=(ray*ray).sum(2);depth=np.full((n,n),np.inf)
    for world in centers:
        center=np.array(world)-[0,0,camera_z];b=-2*(ray*center).sum(2);c=center@center-radius**2
        disc=b*b-4*a*c;hit=disc>0
        values=np.full((n,n),np.inf);values[hit]=(-b[hit]-np.sqrt(disc[hit]))/(2*a[hit])
        depth=np.minimum(depth,values)
    mask=np.isfinite(depth);depth[~mask]=0
    rgb=np.zeros((n,n,3),np.uint8);rgb[mask]=[230,30,50]
    return rgb,depth


def snapshot(stamp,lift,palm_z,camera_z=0):
    tf=dict(translation=[0,0,0],quaternion=[0,0,0,1])
    return dict(tracking_epoch='new-live-trial',streams=dict(rgb=dict(stamp_ns=stamp),depth=dict(stamp_ns=stamp)),
        rgb_k=[200,0,60,0,200,60,0,0,1],base_from_rgb=dict(tf,translation=[0,0,camera_z]),
        base_from_hand=dict(tf,translation=[0,0,palm_z]),palm_from_hand=tf,
        joints=dict(arm_lift_joint=lift,hand_motor_joint=.4 if lift>.25 else .65,
                    arm_flex_joint=-1.7,arm_roll_joint=0,wrist_flex_joint=.1292,wrist_roll_joint=0),
        tracking_transforms=dict(odom_from_base_at_depth=dict(tf,stamp_ns=stamp,source_frame='base_link',target_frame='odom')))


class Tests(unittest.TestCase):
    def setUp(self):
        self.before=(dict(identity_tracking=dict(identity_confirmed=True),detections=[dict(name='apple',confidence=.246)]),
            snapshot(1_000_000_000,.25,.46),dict(center_m=[0,0,.5],radius_m=.04,stamp_ns=1_000_000_000))
        self.after=snapshot(9_000_000_000,.35,.56)
        self.rgb,self.depth=scene()
    def measure(self):return associate(self.rgb,self.depth,self.after,self.before)
    def test_measures_current_depth_not_predicted_center_or_confidence(self):
        value=self.measure();np.testing.assert_allclose(value['center_m'],[0,0,.6],atol=1e-5)
        self.assertAlmostEqual(value['radius_m'],.04,places=5)
        self.assertIsNone(value['detector_confidence']);self.assertFalse(value['identity_confirmed'])
        self.assertFalse(value['motion_authorized'])
        result=pair(self.before,(dict(detections=[]),self.after,value),.4)
        self.assertAlmostEqual(result['object_delta_odom_m'][2],.1,places=5)
        self.assertIsNone(result['confidence'])
    def test_dynamic_camera_height_is_accounted_for(self):
        self.after=snapshot(9_000_000_000,.35,.56,camera_z=.05)
        self.rgb,self.depth=scene(camera_z=.05)
        np.testing.assert_allclose(self.measure()['center_m'],[0,0,.6],atol=1e-5)
    def test_object_left_on_table_cannot_be_moved_by_prediction(self):
        self.rgb,self.depth=scene(centers=((0,0,.5),))
        with self.assertRaises(ValueError):self.measure()
    def test_no_depth_and_flat_red_background_cannot_verify(self):
        self.depth*=0
        with self.assertRaises(ValueError):self.measure()
        self.rgb,self.depth=scene();self.depth[self.depth>0]=.6
        with self.assertRaises(ValueError):self.measure()
    def test_two_current_red_surfaces_not_averaged(self):
        self.rgb,self.depth=scene(centers=((-0.05,0,.6),(.05,0,.6)),radius=.025)
        with self.assertRaisesRegex(ValueError,'unique_verification_rgbd_surface_required'):self.measure()
    def test_no_preclose_identity_no_association(self):
        self.before[0]['identity_tracking']['identity_confirmed']=False
        with self.assertRaisesRegex(ValueError,'confirmed_preclose_identity_required'):self.measure()
    def test_epoch_sync_and_bad_intrinsics_still_rejected(self):
        for mutation in ('epoch','sync','intrinsics'):
            old=copy.deepcopy(self.after)
            if mutation=='epoch':self.after['tracking_epoch']='other'
            elif mutation=='sync':self.after['streams']['rgb']['stamp_ns']+=30_000_001
            else:self.after['rgb_k'][0]=0
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):self.measure()
            self.after=old
    def test_pair_still_rejects_wrong_lift_and_open_hand(self):
        value=self.measure()
        for mutation in ('lift','hand','base'):
            changed=copy.deepcopy(self.after)
            if mutation=='lift':changed['joints']['arm_lift_joint']=.28
            elif mutation=='hand':changed['joints']['hand_motor_joint']=.65
            else:changed['tracking_transforms']['odom_from_base_at_depth']['translation'][0]=.1
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):
                pair(self.before,(dict(detections=[]),changed,value),.4)
    def test_no_contact_or_lift_does_not_run_color_fit(self):
        with patch('pathlib.Path.read_text',return_value=json.dumps(dict(close={},lift={}))), \
                patch('audit_preclose_identity_lift_follow.associate') as fit:
            with self.assertRaisesRegex(ValueError,'actual_contact_and_lift_endpoint_required'):audit('/NO-LIVE-DATA')
            fit.assert_not_called()


if __name__=='__main__':unittest.main()

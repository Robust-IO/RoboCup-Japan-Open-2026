import copy,unittest
import numpy as np
from hand_camera_ray import rotation
from test_pepper_geometry import scene
from pepper_geometry import estimate_source,measure_near
from pepper_transfer_automation import validate_horizontal_placement_hold

class PartialBodyTests(unittest.TestCase):
    def test_occluded_body_adds_real_shoulder_points_without_changing_fit_limits(self):
        rgb,d,s,det,b,c=scene();ref=estimate_source(rgb,d,s,det)
        rgb,d,s,_,b,c=scene(height_delta=.1)
        v,u=np.indices(d.shape);k=np.asarray(s['rgb_k']).reshape(3,3)
        camera=np.stack(((u-k[0,2])*d/k[0,0],(v-k[1,2])*d/k[1,1],d),-1)
        points=camera@rotation(s['base_from_rgb']['quaternion']).T+s['base_from_rgb']['translation'];a=points[:,:,2]-b[2]
        band=(a>.18*ref['height_m']-ref['body_above_support_m'])&(a<.55*ref['height_m']-ref['body_above_support_m'])
        band&=np.linalg.norm(points-b,axis=-1)<ref['body_radius_m']+.012
        depth=d.copy();depth[band&(u%10!=0)]=0
        measured=measure_near(rgb,depth,s,b,c,ref)
        self.assertEqual(measured['body_fit']['curved_shoulder_band_extended_m'],.002)
        self.assertGreaterEqual(measured['body_fit']['points'],60)
        np.testing.assert_allclose(measured['center_m'],b,atol=1e-10)
        self.assertFalse(measured['predicted_center_returned'])
    def test_unoccluded_numeric_fit_keeps_original_band(self):
        rgb,d,s,det,b,c=scene();ref=estimate_source(rgb,d,s,det)
        measured=measure_near(rgb,d,s,b,c,ref)
        self.assertNotIn('curved_shoulder_band_extended_m',measured['body_fit'])

class HorizontalResumeTests(unittest.TestCase):
    def setUp(self):
        self.state=dict(joints=[.267,-1.7,0.,.1292036732051034,0.,.368])
        self.previous=dict(status='stopped_requires_actual_state_review',class_name='ground_pepper',stage='approach_measured_release_region',
            grasp_success_verified=True,release_commands_sent=0,confirmed_stages=['preflight','carry_destination'],
            predecessor_run='original-root',last_fresh_robot_state=self.state)
        self.meta=dict(boot_id='same');self.origin=dict(boot_id='same',run_id='grasp',run_root='original-root')
        self.held=dict(self.origin,class_name='ground_pepper',grasp_lift_verified=True,holding_opening_rad=.368)
        self.plan=dict(class_name='ground_pepper')
    def check(self):return validate_horizontal_placement_hold(self.previous,self.meta,self.origin,self.held,self.plan,self.state,'same')
    def test_original_grasp_binding_survives_a_continuation_run(self):self.check()
    def test_released_wrong_posture_or_wrong_origin_cannot_resume(self):
        self.previous['release_commands_sent']=1
        with self.assertRaises(ValueError):self.check()
        self.previous['release_commands_sent']=0;self.state['joints'][1]=0.
        with self.assertRaises(ValueError):self.check()
        self.state['joints'][1]=-1.7;self.origin['run_id']='another'
        with self.assertRaises(ValueError):self.check()

if __name__=='__main__':unittest.main()

import copy,unittest
from unittest.mock import patch
import numpy as np
from source_table_front import observed_front
from tracked_box_source import reacquire

class SourceFrontTest(unittest.TestCase):
    def scene(self):
        v,u=np.indices((120,200));points=np.stack((12.-.013*v,-7.0+.006*u,np.where(v<81,.5,np.where(v<85,.47,0.))),axis=-1)
        depth=np.ones((120,200));s=dict(streams=dict(depth=dict(stamp_ns=1)),tracking_transforms=dict(map_from_base_at_depth=dict(translation=[0,0,0],quaternion=[0,0,0,1])))
        t=dict(id='white',axis_aligned_bounds=dict(center=dict(x=11.0028,y=-6.409),size=dict(x=.72,y=1.56)))
        plane=dict(plane_map=[0,0,.5],source_depth_stamp_ns=1)
        return points,depth,s,t,plane
    def run_scene(self,points,d,s,t,p):
        with patch('source_table_front.pixel_points',return_value=points),patch('source_table_front.map_pose',return_value=[9.8,-6.9,0.]):return observed_front(d,s,t,p)
    def test_visible_lower_front_face_can_correct_coarse_boundary(self):
        scene=self.scene();before=copy.deepcopy(scene[3]);t,p=self.run_scene(*scene)
        self.assertGreater(p['corrected_front_map_m'],10.9);self.assertEqual(scene[3],before)
        self.assertEqual(t['id'],before['id']);self.assertGreater(p['edge_points'],30)
    def test_floor_behind_mask_is_not_a_front_face(self):
        points,d,s,t,p=self.scene();points[81:,:,2]=0
        with self.assertRaisesRegex(ValueError,'front_face_edge_not_seen'):self.run_scene(points,d,s,t,p)
    def test_depth_holes_are_not_a_front_edge(self):
        points,d,s,t,p=self.scene();d[81:]=0
        with self.assertRaises(ValueError):self.run_scene(points,d,s,t,p)
    def test_image_crop_is_not_a_front_edge(self):
        points,d,s,t,p=self.scene();points[:,:,2]=.5
        with self.assertRaises(ValueError):self.run_scene(points,d,s,t,p)
    def test_mismatched_frame_cannot_change_boundary(self):
        points,d,s,t,p=self.scene();p['source_depth_stamp_ns']=2
        with self.assertRaisesRegex(ValueError,'same_frame'):self.run_scene(points,d,s,t,p)

class TrackedSourceTest(unittest.TestCase):
    def scene(self):
        def snap(stamp,x):return dict(streams=dict(depth=dict(stamp_ns=stamp)),tracking_transforms=dict(odom_from_base_at_depth=dict(translation=[x,0,0],quaternion=[0,0,0,1],stamp_ns=stamp,source_frame='base_link',target_frame='odom'),map_from_base_at_depth=dict(translation=[x,0,0],quaternion=[0,0,0,1],stamp_ns=stamp,source_frame='base_link',target_frame='map')))
        ref=dict(snapshot=snap(1,0),model_detection_verified=True,model=dict(class_name='cigarette',center_m=[1.,0,.51],axis_base=[1,0,0],height_m=.02),appearance={})
        table=dict(axis_aligned_bounds=dict(center=dict(x=1,y=0),size=dict(x=.5,y=.5)))
        return snap(2,.3),ref,table
    def test_requires_prior_model_identity_and_newer_frame(self):
        s,r,t=self.scene();r['model_detection_verified']=False
        with self.assertRaisesRegex(ValueError,'model_confirmed'):reacquire(None,None,s,r,t)
        s,r,t=self.scene();s['streams']['depth']['stamp_ns']=1
        with self.assertRaisesRegex(ValueError,'newer_task_frame'):reacquire(None,None,s,r,t)
    def test_prior_coordinates_only_select_roi_current_depth_is_required(self):
        s,r,t=self.scene()
        with patch('tracked_box_source.measure_near',side_effect=ValueError('current_flat_box_depth_not_seen')):
            with self.assertRaisesRegex(ValueError,'depth_not_seen'):reacquire(None,None,s,r,t)
    def test_reacquisition_uses_current_geometry_and_moved_base(self):
        s,r,t=self.scene();measured=dict(center_m=[.705,0,.51],stamp_ns=2,grasp_point_base_m=[.705,0,.52])
        with patch('tracked_box_source.measure_near',return_value=measured) as fit:
            result=reacquire(None,None,s,r,t)
        np.testing.assert_allclose(fit.call_args.args[3],[.7,0,.51]);self.assertEqual(result['stamp_ns'],2);self.assertEqual(result['center_m'],measured['center_m'])
        self.assertFalse(result['model_detection_used'])
if __name__=='__main__':unittest.main()

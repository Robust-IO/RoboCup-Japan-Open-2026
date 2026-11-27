import copy
import math
import unittest
import cv2
import numpy as np

from canned_grasp_geometry import measure_near
from canned_place_geometry import held_geometry,region
from test_canned_place_geometry import fixture
from tracked_can_cap_geometry import basis
from test_upright_cylinder_geometry import rendered_cylinder


def tilted_scene(angle=14.):
    _,_,snapshot,_,_=rendered_cylinder()
    from hand_camera_ray import rotation
    height=.125;radius=.037;center=np.array([.70,0.,.60])
    axis=np.array([-.35,1.,0.]);axis[:2]*=math.sin(math.radians(angle))/np.linalg.norm(axis[:2]);axis[2]=math.cos(math.radians(angle))
    shape=(240,320);v,u=np.indices(shape);k=np.asarray(snapshot['rgb_k']).reshape(3,3)
    orient=rotation(snapshot['base_from_rgb']['quaternion']);origin=np.array(snapshot['base_from_rgb']['translation'])
    rays=np.stack(((u-k[0,2])/k[0,0],(v-k[1,2])/k[1,1],np.ones(shape)),axis=-1)@orient.T
    offset=origin-center;oa=float(offset@axis);da=rays@axis
    op=offset-oa*axis;dp=rays-da[:,:,None]*axis
    aa=np.sum(dp*dp,axis=-1);bb=2*np.sum(dp*op,axis=-1);cc=float(op@op-radius**2);disc=bb*bb-4*aa*cc
    candidates=[]
    for t in ((-bb-np.sqrt(np.maximum(0,disc)))/(2*aa),(-bb+np.sqrt(np.maximum(0,disc)))/(2*aa)):
        axial=oa+t*da;candidates.append(np.where((disc>=0)&(t>0)&(abs(axial)<=height/2),t,np.inf))
    for z in (-height/2,height/2):
        t=(z-oa)/da;at=op+t[:,:,None]*dp
        candidates.append(np.where((t>0)&(np.sum(at*at,axis=-1)<=radius**2),t,np.inf))
    stack=np.stack(candidates);surface=stack.min(0);part=stack.argmin(0)
    table=(.44-origin[2])/rays[:,:,2];hit=surface<table;depth=np.where(hit,surface,table)
    rgb=np.full((*shape,3),[170,130,100],np.uint8);rgb[hit]=[30,180,60];rgb[hit&(part==3)]=[190,190,190]
    hsv=cv2.cvtColor(np.array([[[30,180,60]]],np.uint8),cv2.COLOR_RGB2HSV)[0,0]
    appearance=dict(class_name='canned_juice',source='actual_verified_can_rgbd',hue=int(hsv[0]),hue_tolerance=6,
        saturation_min=70,value_min=30,silver_saturation_max=80,silver_value_min=110)
    return rgb,depth,snapshot,center,radius,height,axis,appearance


class TiltedCanTests(unittest.TestCase):
    def measure(self,scene):
        rgb,d,s,c,r,h,a,appearance=scene
        return measure_near(d,s,c+[.003,-.003,.002],r,h,rgb=rgb,appearance=appearance)
    def test_true_axis_and_radius_are_measured_at_different_tilts(self):
        for angle in (0.,8.,14.,24.):
            scene=tilted_scene(angle);m=self.measure(scene)
            np.testing.assert_allclose(m['center_m'],scene[3],atol=.003)
            self.assertAlmostEqual(m['tilt_deg'],angle,delta=1.)
            self.assertAlmostEqual(m['surface_radius_m'],scene[4],delta=.002)
            self.assertFalse(m['radius_range_relaxed']);self.assertFalse(m['predicted_center_returned'])
    def test_zero_depth_is_not_a_cap_or_body_measurement(self):
        scene=list(tilted_scene());scene[1]*=0
        with self.assertRaises(ValueError):self.measure(scene)
    def test_wrong_color_cannot_replace_observed_side_with_fingers(self):
        scene=list(tilted_scene());rgb=scene[0];colored=rgb[:,:,1]>rgb[:,:,0]*1.4;rgb[colored]=[230,30,20]
        with self.assertRaises(ValueError):self.measure(scene)
    def test_missing_cap_is_not_filled_from_prediction(self):
        scene=list(tilted_scene());rgb=scene[0];mask=(rgb[:,:,0]==190)&(rgb[:,:,1]==190);scene[1][mask]=0
        with self.assertRaises(ValueError):self.measure(scene)
    def test_missing_body_is_not_proven_by_metal_cap_alone(self):
        scene=list(tilted_scene());rgb=scene[0];mask=(rgb[:,:,0]==30)&(rgb[:,:,1]==180);scene[1][mask]=0
        with self.assertRaises(ValueError):self.measure(scene)
    def nearby_coplanar_wrist_scene(self,*,missing_cap=False):
        scene=list(tilted_scene(0.));rgb,depth,s,center,radius,height,axis,_=scene
        if missing_cap:depth[(rgb[:,:,0]==190)&(rgb[:,:,1]==190)]=0.
        from hand_camera_ray import rotation
        v,u=np.indices(depth.shape);k=np.asarray(s['rgb_k']).reshape(3,3)
        rays=np.stack(((u-k[0,2])/k[0,0],(v-k[1,2])/k[1,1],np.ones(depth.shape)),axis=-1)@rotation(s['base_from_rgb']['quaternion']).T
        origin=np.asarray(s['base_from_rgb']['translation']);capcenter=center+axis*height/2
        distance=((capcenter-origin)@axis)/(rays@axis)
        points=origin+distance[:,:,None]*rays
        radial=np.linalg.norm(points[:,:,:2]-capcenter[:2],axis=-1)
        wrist=(radial>radius+.006)&(radial<radius+.02)&(points[:,:,0]<capcenter[0])
        rgb[wrist]=[220,220,220];depth[wrist]=distance[wrist]
        return scene
    def test_nearby_coplanar_wrist_does_not_expand_the_observed_cap(self):
        scene=self.nearby_coplanar_wrist_scene();m=self.measure(scene)
        np.testing.assert_allclose(m['center_m'],scene[3],atol=.003)
        self.assertLess(m['tilt_deg'],1.)
        self.assertGreater(m['nearby_plane_points_excluded'],0)
        self.assertFalse(m['radius_range_relaxed'])
    def test_nearby_wrist_cannot_replace_a_missing_can_cap(self):
        with self.assertRaises(ValueError):self.measure(self.nearby_coplanar_wrist_scene(missing_cap=True))
    def test_supplied_appearance_is_not_a_new_class_detector(self):
        scene=list(tilted_scene());scene[-1]=dict(scene[-1],source='unverified_color')
        with self.assertRaises(ValueError):self.measure(scene)
    def test_tilt_expands_horizontal_support_and_bottom_extent(self):
        scene=tilted_scene();m=self.measure(scene);a=scene[-2];r=scene[4];h=scene[5]
        expected=h/2*a[2]+r*np.linalg.norm(a[:2])
        self.assertAlmostEqual(m['bottom_vertical_extent_m'],expected,delta=.002)
        self.assertGreater(m['horizontal_support_radius_m'],r+.01)
        self.assertAlmostEqual(m['estimated_bottom_z_m'],scene[3][2]-expected,delta=.003)
    def test_current_bottom_clearance_is_used_by_placement(self):
        ref=dict(can_grasp_and_lift_verified=True,held_center_minus_palm_base_m=[.043,0,.0015],
            can_height_m=.125,can_footprint_radius_m=.037,held_opening_rad=.51,
            can_support_radius_m=.052,estimated_bottom_below_palm_m=.068)
        held=held_geometry(ref);self.assertEqual(held['radius_m'],.052);self.assertEqual(held['surface_radius_m'],.037)
        self.assertEqual(held['bottom_below_palm_m'],.068)
    def test_release_checks_actual_lowest_point_not_half_height(self):
        _,_,s,m,plan,plane=fixture();m['center_m'][2]=.515
        m.update(estimated_lowest_point_base_m=[.71,0,.447],horizontal_support_radius_m=.04)
        check=region(m,s,plan,plane);self.assertAlmostEqual(check['estimated_bottom_gap_m'],.007)
        self.assertEqual(check['table_edge_safety']['object_radius_m'],.04)
    def test_input_pixels_and_calibration_are_not_modified(self):
        scene=tilted_scene();rgb=scene[0].copy();depth=scene[1].copy();s=copy.deepcopy(scene[2]);a=copy.deepcopy(scene[-1])
        self.measure(scene);np.testing.assert_array_equal(scene[0],rgb);np.testing.assert_array_equal(scene[1],depth)
        self.assertEqual(scene[2],s);self.assertEqual(scene[-1],a)
    def test_source_appearance_accepts_detector_geometry_without_unused_top_field(self):
        from held_can_appearance import build_appearance
        rgb,depth,snapshot,center,radius,height,axis,_=tilted_scene(0.)
        geometry=dict(class_name='canned_juice',center_m=center.tolist(),
            footprint_radius_m=radius,height_m=height)
        appearance=build_appearance(rgb,depth,snapshot,geometry)
        self.assertEqual(appearance['source'],'actual_verified_can_rgbd')
        self.assertGreater(appearance['source_body_pixels'],100)


if __name__=='__main__':unittest.main()

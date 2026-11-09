import copy
import json
from pathlib import Path
import subprocess
import sys
import unittest

import cv2
import numpy as np

from canned_grasp_geometry import measure_near
from canned_place_geometry import held_geometry,region,approach_step,body_appearance,placed_expected,verify_placed
from tabletop_support import EDGE_GAP_M,EDGE_UNCERTAINTY_M,GRID_M,RASTER_ALLOWANCE_M
from test_upright_cylinder_geometry import rendered_cylinder


def fixture():
    rgb,depth,s,_,center=rendered_cylinder()
    stamp=s['streams']['depth']['stamp_ns']
    s['tracking_transforms']=dict(map_from_base_at_depth=dict(stamp_ns=stamp,source_frame='base_link',target_frame='map',
        translation=[0,0,0],quaternion=[0,0,0,1]))
    m=measure_near(depth,s,center,.025,.11)
    model=dict(version=1,destination_id='square_low_table#living_room',source_stamp_ns=stamp,
        resolution_m=GRID_M,origin_xy_m=[.2,-.5],mask_rows=['1'*201]*201,
        raster_allowance_m=RASTER_ALLOWANCE_M,basis='observed_table_plane_unknown_is_unsafe')
    plan=dict(class_name='canned_juice',height_m=.11,radius_m=.025,destination_id=model['destination_id'],
        release_xy_map_m=[.7,0],plane_map=[0,0,.44],source_depth_stamp_ns=stamp,observed_table_support=model,
        table_edge_policy=dict(version=1,minimum_outer_gap_m=EDGE_GAP_M,uncertainty_allowance_m=EDGE_UNCERTAINTY_M,
            selection='shortest_reachable_approach_after_clearance_passes'))
    plane=dict(destination_id=model['destination_id'],source_depth_stamp_ns=stamp,plane_map=[0,0,.44])
    return rgb,depth,s,m,plan,plane


class CanPlaceTests(unittest.TestCase):
    def setUp(self):self.rgb,self.depth,self.s,self.m,self.plan,self.plane=fixture()
    def region(self):return region(self.m,self.s,self.plan,self.plane)
    def test_vertical_half_height_not_horizontal_radius(self):
        ref=dict(can_grasp_and_lift_verified=True,held_center_minus_palm_base_m=[.066,0,.001],
            can_height_m=.125,can_footprint_radius_m=.037,held_opening_rad=.48)
        held=held_geometry(ref);self.assertAlmostEqual(held['bottom_below_palm_m'],.0615)
        self.assertNotAlmostEqual(held['bottom_below_palm_m'],held['radius_m'])
    def test_unverified_grasp_cannot_supply_held_dimensions(self):
        with self.assertRaises(ValueError):held_geometry(dict(can_grasp_and_lift_verified=False))
    def test_release_gap_uses_can_height(self):
        self.m['center_m'][2]=.515
        self.assertAlmostEqual(self.region()['estimated_bottom_gap_m'],.02)
        self.assertTrue(self.region()['ready'])
    def test_functional_support_accepts_different_safe_xy(self):
        self.m['center_m'][0]+=.08
        r=self.region();self.assertTrue(r['ready']);self.assertGreater(r['planned_xy_error_m'],.07)
        self.assertFalse(r['planned_xy_is_success_gate'])
    def test_edge_and_unknown_support_cannot_release(self):
        self.m['center_m'][0]=.22
        self.assertFalse(self.region()['ready'])
        self.plan['observed_table_support']['mask_rows']=['0'*201]*201
        self.m['center_m'][0]=.7;self.assertFalse(self.region()['ready'])
    def test_invalid_height_does_not_send_xy_workaround(self):
        self.m['center_m'][2]=.70
        self.assertFalse(self.region()['ready'])
        with self.assertRaisesRegex(ValueError,'height'):approach_step(self.m,self.s,self.plan,self.plane)
    def test_new_surface_class_or_time_mismatch_is_rejected(self):
        for kind in ('class','shape','stamp','plane','destination'):
            rgb,d,s,m,p,plane=fixture()
            if kind=='class':m['class_name']='apple'
            if kind=='shape':m['shape']='sphere'
            if kind=='stamp':m['stamp_ns']+=1
            if kind=='plane':plane['source_depth_stamp_ns']+=1
            if kind=='destination':plane['destination_id']='other'
            with self.subTest(kind=kind),self.assertRaises(ValueError):region(m,s,p,plane)
    def test_changed_table_height_is_not_same_destination(self):
        self.plane['plane_map'][2]+=.03
        with self.assertRaises(ValueError):self.region()
    def test_full_finite_approach_has_no_fixed_distance_cap(self):
        self.m['center_m'][0]=-.8
        step=approach_step(self.m,self.s,self.plan,self.plane)
        self.assertFalse(step['aligned']);self.assertGreater(step['delta_m'][0],1.)
        self.assertIsNone(step['fixed_translation_limit_m'])
    def test_actual_release_xy_is_tracking_roi_after_release(self):
        self.plan['actual_release_xy_map_m']=[.78,.04]
        np.testing.assert_allclose(placed_expected(self.s,self.plan),[.78,.04,.495])
    def color_model(self):
        hsv=cv2.cvtColor(np.array([[[30,180,60]]],np.uint8),cv2.COLOR_RGB2HSV)[0,0]
        return dict(source='actual_verified_can_rgbd',hue=int(hsv[0]),hue_tolerance=6,saturation_min=70,value_min=30)
    def test_actual_color_pixels_on_depth_can_surface_continue_identity(self):
        result=body_appearance(self.rgb,self.depth,self.s,self.m,self.color_model())
        self.assertGreater(result['matching_actual_depth_body_pixels'],30)
        self.assertFalse(result['new_object_identity_created'])
    def test_no_depth_or_wrong_color_is_not_appearance_evidence(self):
        for rgb,d in ((self.rgb,self.depth*0),(np.full_like(self.rgb,[230,30,25]),self.depth)):
            with self.assertRaises(ValueError):body_appearance(rgb,d,self.s,self.m,self.color_model())
    def rows(self):
        rows=[]
        for i in range(3):
            s=copy.deepcopy(self.s);m=copy.deepcopy(self.m);plane=copy.deepcopy(self.plane);stamp=1_000_000_000+i*200_000_000
            s['streams']['depth']['stamp_ns']=stamp;s['tracking_transforms']['map_from_base_at_depth']['stamp_ns']=stamp
            m['stamp_ns']=stamp;plane['source_depth_stamp_ns']=stamp
            rows.append(dict(snapshot=s,measurement=m,plane=plane))
        return rows
    def test_three_stable_surface_frames_prove_placement_not_full_automation(self):
        result=verify_placed(self.rows(),self.plan,[0,0,0,-np.pi/2,0,.65],cleanup_completed=True)
        self.assertTrue(result['placement_success_verified']);self.assertFalse(result['full_uninterrupted_transfer_verified'])
        self.assertFalse(result['task_finished_sent'])
    def test_open_hand_alone_or_one_frame_does_not_prove_placement(self):
        with self.assertRaises(ValueError):verify_placed(self.rows()[:1],self.plan,[0,0,0,0,0,.65],cleanup_completed=True)
        with self.assertRaises(ValueError):verify_placed(self.rows(),self.plan,[0,0,0,0,0,.48],cleanup_completed=True)
    def test_floating_or_falling_can_is_not_placed(self):
        rows=self.rows();rows[2]['measurement']['center_m'][2]+=.04
        with self.assertRaises(ValueError):verify_placed(rows,self.plan,[0,0,0,0,0,.65],cleanup_completed=True)
    def test_cleanup_failure_is_separate_from_physical_placement(self):
        result=verify_placed(self.rows(),self.plan,[.2,-1.7,0,.1292,0,.65],cleanup_completed=False)
        self.assertTrue(result['placement_success_verified']);self.assertFalse(result['cleanup_completed'])
    def test_default_cli_has_no_actions_or_ros(self):
        result=subprocess.run([sys.executable,str(Path(__file__).with_name('run_canned_place_trial.py'))],capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr);value=json.loads(result.stdout)
        self.assertFalse(value['robot_commands_sent']);self.assertFalse(value['grasp_supported']);self.assertFalse(value['navigation_supported'])


if __name__=='__main__':unittest.main()

import copy,unittest
import numpy as np
from flat_box_place_geometry import region,approach_step,verify_placed
from tabletop_support import EDGE_GAP_M,EDGE_UNCERTAINTY_M,GRID_M,RASTER_ALLOWANCE_M

def fixture():
 stamp=1000000000;destination='square_low_table#living_room'
 s=dict(streams=dict(depth=dict(stamp_ns=stamp)),tracking_transforms=dict(map_from_base_at_depth=dict(
  stamp_ns=stamp,source_frame='base_link',target_frame='map',translation=[0,0,0],quaternion=[0,0,0,1])))
 support=dict(version=1,destination_id=destination,source_stamp_ns=stamp,resolution_m=GRID_M,origin_xy_m=[.2,-.5],
  mask_rows=['1'*201]*201,raster_allowance_m=RASTER_ALLOWANCE_M,basis='observed_table_plane_unknown_is_unsafe')
 plan=dict(class_name='cigarette',height_m=.0225,radius_m=.049,destination_id=destination,release_xy_map_m=[.7,0],
  plane_map=[0,0,.44],source_depth_stamp_ns=stamp,observed_table_support=support,
  table_edge_policy=dict(version=1,minimum_outer_gap_m=EDGE_GAP_M,uncertainty_allowance_m=EDGE_UNCERTAINTY_M,
  selection='shortest_reachable_approach_after_clearance_passes'))
 m=dict(class_name='cigarette',shape='flat_cuboid',center_m=[.7,0,.47125],height_m=.0225,stamp_ns=stamp)
 plane=dict(destination_id=destination,source_depth_stamp_ns=stamp,plane_map=[0,0,.44])
 return s,m,plan,plane

class BoxPlaceTest(unittest.TestCase):
 def actual_rows(self,x):
  s,m,p,t=fixture();p['outcome_profile']='outcome_v1';p['table_edge_policy']=dict(version=2,profile='outcome_v1',minimum_outer_gap_m=.02,uncertainty_allowance_m=.01)
  rows=[]
  for i in range(2):
   ss,mm,tt=copy.deepcopy((s,m,t));stamp=1000000000+i*200000000;ss['streams']['depth']['stamp_ns']=stamp;ss['tracking_transforms']['map_from_base_at_depth']['stamp_ns']=stamp
   mm['stamp_ns']=stamp;mm['center_m'][0]=x;tt['source_depth_stamp_ns']=stamp;rows.append(dict(snapshot=ss,measurement=mm,plane=tt))
  return rows,p
 def test_settled_supported_object_does_not_need_full_planning_standoff(self):
  rows,p=self.actual_rows(.285);result=verify_placed(rows,p,[0,0,0,-np.pi/2,0,.65],cleanup_completed=True)
  self.assertTrue(result['placement_success_verified']);self.assertFalse(result['planning_edge_margin_met'])
  self.assertFalse(result['planning_edge_margin_is_post_release_gate'])
  self.assertFalse(region(rows[0]['measurement'],rows[0]['snapshot'],p,rows[0]['plane'])['table_edge_safety']['safe'])
 def test_post_release_full_footprint_and_uncertainty_still_required(self):
  for x in (.24,.275):
   rows,p=self.actual_rows(x)
   with self.subTest(x=x),self.assertRaisesRegex(ValueError,'not_supported'):verify_placed(rows,p,[0,0,0,-np.pi/2,0,.65],cleanup_completed=True)
 def test_bottom_contact_and_stability_remain_required(self):
  rows,p=self.actual_rows(.7);rows[1]['measurement']['center_m'][2]+=.06
  with self.assertRaises(ValueError):verify_placed(rows,p,[0,0,0,-np.pi/2,0,.65],cleanup_completed=True)
 def test_flat_thickness_defines_bottom(self):
  s,m,p,t=fixture();r=region(m,s,p,t);self.assertAlmostEqual(r['estimated_bottom_gap_m'],.02);self.assertTrue(r['ready'])
 def test_safe_point_need_not_equal_planned_point(self):
  s,m,p,t=fixture();m['center_m'][0]+=.08;r=region(m,s,p,t);self.assertTrue(r['ready']);self.assertFalse(r['planned_xy_is_success_gate'])
 def test_box_corners_must_stay_on_support(self):
  s,m,p,t=fixture();m['center_m'][0]=.25;self.assertFalse(region(m,s,p,t)['ready'])
 def test_single_full_approach_has_no_displacement_cap(self):
  s,m,p,t=fixture();m['center_m'][0]=-.8;self.assertGreater(approach_step(m,s,p,t)['delta_m'][0],1.)
 def test_actual_table_and_box_need_same_frame(self):
  s,m,p,t=fixture();t['source_depth_stamp_ns']+=1
  with self.assertRaises(ValueError):region(m,s,p,t)
 def test_three_independent_stable_placed_frames(self):
  s,m,p,t=fixture();rows=[]
  for i in range(3):
   ss,mm,tt=copy.deepcopy((s,m,t));stamp=1000000000+i*200000000
   ss['streams']['depth']['stamp_ns']=stamp;ss['tracking_transforms']['map_from_base_at_depth']['stamp_ns']=stamp
   mm['stamp_ns']=stamp;tt['source_depth_stamp_ns']=stamp;rows.append(dict(snapshot=ss,measurement=mm,plane=tt))
  self.assertTrue(verify_placed(rows,p,[0,0,0,-np.pi/2,0,.65],cleanup_completed=True)['placement_success_verified'])
  rows[-1]['measurement']['center_m'][2]+=.06
  with self.assertRaises(ValueError):verify_placed(rows,p,[0,0,0,-np.pi/2,0,.65],cleanup_completed=True)
if __name__=='__main__':unittest.main()

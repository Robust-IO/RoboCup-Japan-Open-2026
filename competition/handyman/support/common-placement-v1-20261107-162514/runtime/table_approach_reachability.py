"""Manipulation reach constraints; leaves Nav2 and motion speed unchanged.

The .22m base envelope matches the supplied navigation circle. Treat the whole
destination footprint as unavailable to the base, even where legs leave gaps.
It is a conservative planar envelope, not a full robot collision certificate.
"""
import math
import numpy as np
from apple_transfer_auto_plan import (footprint,map_pose,map_point,depth_in_map,clear_patch)
from tabletop_support import build_support,TableSupport,PLANNING_RESERVE_M,EDGE_GAP_M,EDGE_UNCERTAINTY_M
from hand_camera_ray import rotation
from inverse_table_place_plan import release_height
from pregrasp_geometry import expected_horizontal_center

BASE_RADIUS_M=.22
BASE_CLEARANCE_M=.015

def base_path(start,delta,table):
 start=np.asarray(start,float);delta=np.asarray(delta,float)
 if start.shape!=(3,) or delta.shape!=(3,) or not np.isfinite([*start,*delta]).all() or abs(delta[2])>1e-6:raise ValueError('fixed_heading_translation_required')
 center,half=footprint(table);c,s=math.cos(start[2]),math.sin(start[2])
 end=start[:2]+np.array([[c,-s],[s,c]])@delta[:2]
 lo=center-half-BASE_RADIUS_M-BASE_CLEARANCE_M;hi=center+half+BASE_RADIUS_M+BASE_CLEARANCE_M
 # Segment/slab intersection, including any unsafe intermediate pose.
 enter,leave=0.,1.;v=end-start[:2]
 for i in range(2):
  if abs(v[i])<1e-12:
   if not lo[i]<=start[i]<=hi[i]:return dict(ready=True,start_xy_m=start[:2].tolist(),end_xy_m=end.tolist(),base_radius_m=BASE_RADIUS_M,clearance_m=BASE_CLEARANCE_M)
  else:
   a,b=sorted(((lo[i]-start[i])/v[i],(hi[i]-start[i])/v[i]));enter=max(enter,a);leave=min(leave,b)
 collision=enter<=leave
 return dict(ready=not collision,start_xy_m=start[:2].tolist(),end_xy_m=end.tolist(),base_radius_m=BASE_RADIUS_M,
  clearance_m=BASE_CLEARANCE_M,reason='base_sweeps_destination_footprint' if collision else 'base_outside_destination_footprint',
  full_translation_m=float(np.linalg.norm(delta[:2])),fixed_distance_cap_m=None)

def hand_path(points,start_xy,end_xy,plane,palm_height,radius):
 """Visible obstacles along full held-object/pad sweep, not just endpoint."""
 start=np.asarray(start_xy);end=np.asarray(end_xy);v=end-start;d=points[:,:2]-start
 t=np.clip(d@v/max(float(v@v),1e-12),0.,1.);near=np.linalg.norm(d-t[:,None]*v,axis=1)<max(.105,radius+.025)
 z=points[:,2]-(np.c_[points[:,:2],np.ones(len(points))]@plane)
 obstacles=near&(z>.015)&(points[:,2]<palm_height+.09)
 return dict(ready=not bool(obstacles.any()),obstacle_points=int(obstacles.sum()),
  corridor_radius_m=max(.105,radius+.025),basis='current_depth_visible_swept_hand_corridor',unseen_space_not_certified=True)

def plan_placement(depth,snapshot,table,plane,held,config,task_id,*,minimum_shift_m=0.,held_volume=None,outcome_policy=None):
 center,half=footprint(table);points=depth_in_map(depth,snapshot);coef=np.asarray(plane['plane_map']);stamp=snapshot['streams']['depth']['stamp_ns']
 if plane['destination_id']!=table['id'] or plane['source_depth_stamp_ns']!=stamp:raise ValueError('same_frame_destination_plane_required')
 if not math.isfinite(held['radius_m']) or held['radius_m']<=0:raise ValueError('verified_held_support_radius_required')
 allowances=dict(edge_gap_m=outcome_policy['placement']['outer_edge_gap_m'],uncertainty_m=outcome_policy['placement']['position_uncertainty_m']) if outcome_policy else {}
 support_model=build_support(points,center,half,coef,table['id'],stamp);support=TableSupport(support_model,table['id'],**allowances)
 reserve=outcome_policy['placement']['planning_reserve_m'] if outcome_policy else PLANNING_RESERVE_M
 positions=support.safe_centers(held['radius_m'],reserve);r=rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion']);start=map_pose(snapshot)
 obstacle_points=points
 if held_volume is not None:
  vc=map_point(snapshot,np.asarray(held_volume['center_m']));axes=r@np.asarray(held_volume['axes_base'])
  own=(abs((points-vc)@axes)<=np.asarray(held_volume['sizes_m'])/2+.008).all(1)
  obstacle_points=points[~own]
 candidates=[];rejected=[];policy=config['policy']
 for xy in positions:
  height=float(np.r_[xy,1.]@coef);palm_height=release_height(height,held['bottom_below_palm_m'],policy['release_gap_m'])
  predicted=map_point(snapshot,expected_horizontal_center(palm_height,held['center_minus_palm_m']))
  delta=r.T@np.r_[xy-predicted[:2],0.];distance=float(np.linalg.norm(delta[:2]))
  if distance<minimum_shift_m or (not outcome_policy and delta[0]<=0):continue
  reach=base_path(start,[*delta[:2],0.],table)
  if not reach['ready']:
   if len(rejected)<12:rejected.append(dict(xy_map_m=xy.tolist(),approach_distance_m=distance,reachability=reach))
   continue
  candidates.append((distance,xy,palm_height,height,predicted,delta,reach))
 candidates.sort(key=lambda a:(a[0],a[1][0],a[1][1]));checked=[]
 for distance,xy,palm_height,height,predicted,delta,reach in candidates:
  patch=clear_patch(obstacle_points,xy,coef,max(policy['patch_radius_m'],held['radius_m']+.01))
  if outcome_policy:
   # Disk support/edge clearance is already checked independently. Eight
   # extra sectors around a larger patch are diagnostic, not a second veto.
   patch['clear']=patch['table_surface_pixels']>=150 and patch['obstacle_pixels']<outcome_policy['placement']['minimum_obstacle_points']
   patch['sector_check_is_diagnostic']=True
  swept=hand_path(obstacle_points,predicted[:2],xy,coef,palm_height,held['radius_m'])
  if len(checked)<32:checked.append(dict(xy_map_m=xy.tolist(),approach_distance_m=distance,**patch,swept_hand=swept))
  if not patch['clear'] or not swept['ready']:continue
  return dict(task_id=task_id,destination_id=table['id'],destination_room=table['room'],release_xy_map_m=xy.tolist(),palm_height_m=palm_height,
   plane_map=coef.tolist(),table_height_at_release_m=height,footprint_center=center.tolist(),footprint_half_size=half.tolist(),
   held_center_minus_palm_m=held['center_minus_palm_m'],holding_opening_rad=held['holding_opening_rad'],radius_m=held['radius_m'],
   patch=patch,checked_candidates=checked,rejected_unreachable_candidates=rejected,base_reachability=reach,swept_hand=swept,
   table_edge_policy=(dict(version=2,profile='outcome_v1',minimum_outer_gap_m=support.edge_gap_m,uncertainty_allowance_m=support.uncertainty_m,selection='shortest_reachable_approach_after_clearance_passes') if outcome_policy else dict(version=1,minimum_outer_gap_m=EDGE_GAP_M,uncertainty_allowance_m=EDGE_UNCERTAINTY_M,selection='shortest_reachable_approach_after_clearance_passes')),
   observed_table_support=support_model,table_edge_safety=support.query(xy,held['radius_m']),planned_approach_distance_m=distance,
   source_depth_stamp_ns=stamp,motion_authorized=False,navigation_parameters_changed=False)
 raise ValueError('no_clear_placement_patch_reachable_from_current_side')

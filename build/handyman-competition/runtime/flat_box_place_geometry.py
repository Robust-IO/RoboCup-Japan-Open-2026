"""Flat box support acceptance; no cylinder dimensions or identity aliases."""
import numpy as np
from apple_transfer_auto_plan import map_point,map_pose
from hand_camera_ray import rotation
from tabletop_support import support_from_plan
from task_outcome_policy import RELEASE_BOTTOM_GAP_M,PLACED_BOTTOM_GAP_M,PLACED_CENTER_SPREAD_M,hand_is_open

def region(measurement,snapshot,plan,current_plane):
 if (plan.get('class_name')!='cigarette' or measurement.get('class_name')!='cigarette'
     or measurement.get('shape')!='flat_cuboid' or measurement['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']
     or current_plane['source_depth_stamp_ns']!=measurement['stamp_ns'] or current_plane['destination_id']!=plan['destination_id']):
  raise ValueError('same_frame_box_and_table_required')
 center=np.asarray(measurement['center_m']);height=measurement['height_m']
 height_tolerance=.015 if plan.get('outcome_profile')=='outcome_v1' else .005
 if center.shape!=(3,) or not np.isfinite([*center,height]).all() or height<=0:raise ValueError('box_dimensions_changed')
 mapped=map_point(snapshot,center)
 bottom=map_point(snapshot,np.asarray(measurement.get('estimated_lowest_point_base_m',center-[0,0,height/2])))
 table=float(np.r_[bottom[:2],1.]@current_plane['plane_map'])
 if abs(table-np.r_[bottom[:2],1.]@plan['plane_map'])>.02:raise ValueError('table_height_changed')
 radius=max(float(plan['radius_m']),float(measurement.get('horizontal_support_radius_m',0.)))
 edge=support_from_plan(plan).query(mapped[:2],radius);gap=float(bottom[2]-table)
 return dict(ready=bool(edge['safe'] and RELEASE_BOTTOM_GAP_M[0]<=gap<=RELEASE_BOTTOM_GAP_M[1]),
  center_map_m=mapped.tolist(),bottom_map_m=bottom.tolist(),estimated_bottom_gap_m=gap,
  table_height_map_m=table,table_edge_safety=edge,planned_xy_error_m=float(np.linalg.norm(mapped[:2]-plan['release_xy_map_m'])),
  planned_xy_is_success_gate=False,box_half_diagonal_support_radius_m=radius,motion_authorized=False)

def approach_step(measurement,snapshot,plan,current_plane):
 check=region(measurement,snapshot,plan,current_plane)
 if check['ready']:return dict(aligned=True,delta_m=[0.,0.,0.],region=check)
 if not RELEASE_BOTTOM_GAP_M[0]<=check['estimated_bottom_gap_m']<=RELEASE_BOTTOM_GAP_M[1]:raise ValueError('box_release_height_not_aligned')
 r=rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
 delta=r.T@np.r_[np.asarray(plan['release_xy_map_m'])-check['center_map_m'][:2],0.]
 if not np.isfinite(delta).all() or np.linalg.norm(delta[:2])<.001:raise ValueError('no_valid_placement_translation')
 return dict(aligned=False,delta_m=[float(delta[0]),float(delta[1]),0.],region=check,fixed_translation_limit_m=None,full_measured_translation=True)

def placed_expected(snapshot,plan):
 xy=np.asarray(plan.get('actual_release_xy_map_m',plan['release_xy_map_m']));z=float(np.r_[xy,1.]@plan['plane_map'])+plan['height_m']/2
 map_pose(snapshot);tf=snapshot['tracking_transforms']['map_from_base_at_depth']
 return rotation(tf['quaternion']).T@(np.r_[xy,z]-tf['translation'])

def verify_placed(rows,plan,joints,*,cleanup_completed):
 from outcome_transfer_policy import load
 policy=load()['verification'];required=policy['placed_frames'] if plan.get('outcome_profile')=='outcome_v1' else 3
 if len(rows)<required or not hand_is_open(joints[5]):raise ValueError('independent_placed_frames_and_open_hand_required')
 stamps=[r['measurement']['stamp_ns'] for r in rows]
 if any(b-a<150_000_000 for a,b in zip(stamps,stamps[1:])):raise ValueError('distinct_placed_observations_required')
 checks=[region(r['measurement'],r['snapshot'],plan,r['plane']) for r in rows]
 # Planning keeps the full edge gap plus uncertainty before release. Once
 # actually settled, require full observed support including uncertainty,
 # not exact reproduction of the extra planning standoff after withdrawal.
 outcome=plan.get('outcome_profile')=='outcome_v1'
 def supported(c):
  edge=c['table_edge_safety']
  return edge['outer_clearance_lower_estimate_m']>=edge['uncertainty_allowance_m'] if outcome else edge['safe']
 if any(not supported(c) or abs(c['estimated_bottom_gap_m'])>PLACED_BOTTOM_GAP_M for c in checks):raise ValueError('box_not_supported_on_destination')
 centers=np.asarray([c['center_map_m'] for c in checks]);spread=float(np.max(np.linalg.norm(centers-centers.mean(0),axis=1)))
 if spread>PLACED_CENTER_SPREAD_M:raise ValueError('placed_box_not_stable')
 return dict(placement_success_verified=True,object='cigarette',destination_id=plan['destination_id'],checks=checks,
  center_spread_m=spread,hand_open_verified=True,cleanup_completed=cleanup_completed,planned_xy_is_success_gate=False,
  task_finished_sent=False,full_uninterrupted_transfer_verified=False,independent_observations=len(rows),
  planning_edge_margin_is_post_release_gate=not outcome,
  planning_edge_margin_met=all(c['table_edge_safety']['safe'] for c in checks),
  actual_supported_footprint_with_uncertainty=True)

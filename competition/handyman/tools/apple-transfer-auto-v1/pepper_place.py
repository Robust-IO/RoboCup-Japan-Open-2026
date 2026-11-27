"""Pepper placement from actual body/cap depth and measured tabletop support."""
import numpy as np
from apple_transfer_auto_plan import plan_placement,map_point
from hand_camera_ray import rotation
from tabletop_support import support_from_plan
from task_outcome_policy import RELEASE_BOTTOM_GAP_M,PLACED_BOTTOM_GAP_M,PLACED_CENTER_SPREAD_M,hand_is_open
from pepper_geometry import NAME,surface_extent


def extension_clearance(predicted,snapshot,plane,plan):
    """Same pre-extension clearance, projected onto the current heading."""
    f=rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])[:2,0]
    def near(center,half):return float(np.asarray(center)@f-np.asarray(half)@abs(f))
    observed_center=(np.asarray(plane['observed_xy_min'])+plane['observed_xy_max'])/2
    observed_half=(np.asarray(plane['observed_xy_max'])-plane['observed_xy_min'])/2
    front=min(near(observed_center,observed_half),near(plan['footprint_center'],plan['footprint_half_size']))
    tip=float(np.asarray(predicted)[:2]@f)+max(plan['radius_m'],.08)
    return dict(ready=bool(tip<front-.03),front_along_heading_m=front,
        tip_along_heading_m=tip,clearance_m=front-tip,minimum_clearance_m=.03)


def make_plan(depth,snapshot,table,plane,reference,config,task_id):
    model=reference['model'];m=reference['last_horizontal_measurement'];offset=reference['body_minus_palm_horizontal_m']
    held=dict(center_minus_palm_m=offset,radius_m=m['horizontal_support_radius_m'],
        holding_opening_rad=reference['holding_opening_rad'],
        bottom_below_palm_m=m['bottom_vertical_extent_m']-offset[2])
    value=plan_placement(depth,snapshot,table,plane,held,config,task_id)
    return dict(value,class_name=NAME,body_radius_m=model['body_radius_m'],height_m=model['height_m'],
        body_above_support_m=model['body_above_support_m'],cap_radius_m=model['cap_radius_m'],
        cap_minus_body_horizontal_m=(np.asarray(m['cap_center_m'])-m['center_m']).tolist())


def region(measurement,snapshot,plan,current_plane):
    if (measurement.get('class_name')!=NAME or plan.get('class_name')!=NAME
            or measurement['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']
            or current_plane['source_depth_stamp_ns']!=measurement['stamp_ns']
            or current_plane['destination_id']!=plan['destination_id']):raise ValueError('same_frame_pepper_and_support_required')
    center=map_point(snapshot,np.asarray(measurement['center_m']))
    bottom=map_point(snapshot,np.asarray(measurement['estimated_lowest_point_base_m']))
    height=float(np.r_[bottom[:2],1.]@current_plane['plane_map'])
    gap=float(bottom[2]-height)
    edge=support_from_plan(plan).query(center[:2],measurement['horizontal_support_radius_m'])
    return dict(ready=bool(edge['safe'] and RELEASE_BOTTOM_GAP_M[0]<=gap<=RELEASE_BOTTOM_GAP_M[1]),
        center_map_m=center.tolist(),bottom_map_m=bottom.tolist(),estimated_bottom_gap_m=gap,
        table_height_map_m=height,table_edge_safety=edge,planned_xy_is_success_gate=False,motion_authorized=False)


def approach_step(measurement,snapshot,plan,plane):
    check=region(measurement,snapshot,plan,plane)
    if check['ready']:return dict(aligned=True,delta_m=[0.,0.,0.],region=check)
    if not RELEASE_BOTTOM_GAP_M[0]<=check['estimated_bottom_gap_m']<=RELEASE_BOTTOM_GAP_M[1]:
        raise ValueError('current_pepper_release_height_not_aligned')
    delta=np.asarray(plan['release_xy_map_m'])-check['center_map_m'][:2]
    r=rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
    local=r.T@np.r_[delta,0.]
    return dict(aligned=False,delta_m=[float(local[0]),float(local[1]),0.],region=check)


def expected_placed(snapshot,plan):
    xy=np.asarray(plan['actual_release_xy_map_m']);height=float(np.r_[xy,1.]@plan['plane_map'])
    body=np.r_[xy,height+plan['body_above_support_m']]
    tf=snapshot['tracking_transforms']['map_from_base_at_depth'];r=rotation(tf['quaternion']);t=np.asarray(tf['translation'])
    cap=body+np.asarray(plan['actual_cap_minus_body_map_m'] if 'actual_cap_minus_body_map_m' in plan
        else r@plan['cap_minus_body_horizontal_m'])
    return r.T@(body-t),r.T@(cap-t)


def verify_placed(rows,plan,joints,*,cleanup_completed):
    if len(rows)!=3 or not hand_is_open(joints[5]):raise ValueError('three_pepper_frames_and_open_hand_required')
    stamps=[r['measurement']['stamp_ns'] for r in rows]
    if any(b-a<150_000_000 for a,b in zip(stamps,stamps[1:])):raise ValueError('distinct_placed_pepper_frames_required')
    checks=[region(r['measurement'],r['snapshot'],plan,r['plane']) for r in rows]
    if any(not r['table_edge_safety']['safe'] or abs(r['estimated_bottom_gap_m'])>PLACED_BOTTOM_GAP_M for r in checks):
        raise ValueError('pepper_not_supported_on_destination')
    centers=np.asarray([r['center_map_m'] for r in checks]);spread=float(np.max(np.linalg.norm(centers-centers.mean(0),axis=1)))
    if spread>PLACED_CENTER_SPREAD_M:raise ValueError('placed_pepper_not_stable')
    return dict(placement_success_verified=True,class_name=NAME,destination_id=plan['destination_id'],
        checks=checks,center_spread_m=spread,cleanup_completed=cleanup_completed,task_finished_sent=False)

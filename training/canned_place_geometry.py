"""Measured upright-can placement. Pure geometry; no controller or authority."""
import numpy as np
import cv2

from apple_transfer_auto_plan import map_point,map_pose,plan_placement
from hand_camera_ray import rotation
from held_can_appearance import pixel_points
from tabletop_support import support_from_plan
from task_outcome_policy import RELEASE_BOTTOM_GAP_M,PLACED_BOTTOM_GAP_M,PLACED_CENTER_SPREAD_M,hand_is_open


def held_geometry(reference):
    if reference.get('can_grasp_and_lift_verified') is not True:raise ValueError('verified_can_grasp_required')
    offset=np.asarray(reference['held_center_minus_palm_base_m'],float)
    height=reference['can_height_m'];radius=reference['can_footprint_radius_m'];opening=reference['held_opening_rad']
    if (offset.shape!=(3,) or not np.isfinite([*offset,height,radius,opening]).all()
            or not .035<=height<=.20 or not .01<=radius<=.06 or not .02<opening<=.62):
        raise ValueError('invalid_verified_can_geometry')
    bottom=float(reference.get('estimated_bottom_below_palm_m',height/2-offset[2]))
    if not 0<bottom<.2:raise ValueError('invalid_can_bottom_offset')
    return dict(center_minus_palm_m=offset.tolist(),radius_m=reference.get('can_support_radius_m',radius),
        surface_radius_m=radius,holding_opening_rad=opening,bottom_below_palm_m=bottom,height_m=height)


def make_plan(depth,snapshot,table,plane,reference,config,task_id):
    held=held_geometry(reference)
    plan=plan_placement(depth,snapshot,table,plane,held,config,task_id)
    return dict(plan,class_name='canned_juice',shape='upright_cylinder',height_m=held['height_m'],
        surface_radius_m=held['surface_radius_m'],bottom_below_palm_m=held['bottom_below_palm_m'],vertical_size_is_not_footprint_radius=True)


def region(measurement,snapshot,plan,current_plane):
    if (plan.get('class_name')!='canned_juice' or measurement.get('class_name')!='canned_juice'
            or measurement.get('shape')!='upright_cylinder' or measurement.get('stamp_ns')!=snapshot['streams']['depth']['stamp_ns']
            or current_plane['source_depth_stamp_ns']!=measurement['stamp_ns']
            or current_plane['destination_id']!=plan['destination_id']):raise ValueError('same_frame_can_and_table_required')
    center=np.asarray(measurement['center_m'],float);height=measurement['height_m'];radius=measurement['footprint_radius_m']
    if (center.shape!=(3,) or not np.isfinite([*center,height,radius]).all()
            or abs(height-plan['height_m'])>.005):raise ValueError('can_dimensions_changed')
    mapped=map_point(snapshot,center)
    bottom=map_point(snapshot,np.asarray(measurement.get('estimated_lowest_point_base_m',center-[0,0,height/2])))
    table_height=float(np.r_[bottom[:2],1.]@current_plane['plane_map'])
    old_height=float(np.r_[bottom[:2],1.]@plan['plane_map'])
    if abs(table_height-old_height)>.02:raise ValueError('destination_table_height_changed')
    edge=support_from_plan(plan).query(mapped[:2],measurement.get('horizontal_support_radius_m',radius));gap=float(bottom[2]-table_height)
    return dict(ready=bool(edge['safe'] and RELEASE_BOTTOM_GAP_M[0]<=gap<=RELEASE_BOTTOM_GAP_M[1]),
        center_map_m=mapped.tolist(),bottom_map_m=bottom.tolist(),estimated_bottom_gap_m=gap,
        table_height_map_m=table_height,table_edge_safety=edge,
        planned_xy_error_m=float(np.linalg.norm(mapped[:2]-plan['release_xy_map_m'])),
        planned_xy_is_success_gate=False,bottom_from_cylinder_half_height=True,motion_authorized=False)


def approach_step(measurement,snapshot,plan,current_plane):
    check=region(measurement,snapshot,plan,current_plane)
    if check['ready']:return dict(aligned=True,delta_m=[0.,0.,0.],region=check)
    if not RELEASE_BOTTOM_GAP_M[0]<=check['estimated_bottom_gap_m']<=RELEASE_BOTTOM_GAP_M[1]:
        raise ValueError('current_can_release_height_not_aligned')
    rot=rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
    delta=rot.T@np.r_[np.asarray(plan['release_xy_map_m'])-check['center_map_m'][:2],0.]
    if not np.isfinite(delta).all() or np.linalg.norm(delta[:2])<.001:raise ValueError('no_valid_measured_placement_translation')
    return dict(aligned=False,delta_m=[float(delta[0]),float(delta[1]),0.],region=check,
        fixed_translation_limit_m=None,full_measured_translation=True)


def body_appearance(rgb,depth,snapshot,measurement,appearance):
    """Confirm actual measured can pixels continue the known held object's color."""
    if appearance.get('source')!='actual_verified_can_rgbd':raise ValueError('verified_body_appearance_required')
    p=pixel_points(depth,snapshot);c=np.asarray(measurement['center_m']);h=measurement['height_m'];r=measurement['footprint_radius_m']
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    axis=np.asarray(measurement.get('axis_base_m',[0.,0.,1.]),float)
    delta3=p-c;axial=delta3@axis;radial=np.linalg.norm(delta3-axial[:,:,None]*axis,axis=2)
    mask=(np.isfinite(depth)&(depth>0)&(radial<r+.004)&(axial>-.35*h)&(axial<.45*h))
    delta=np.abs(hsv[:,:,0].astype(float)-appearance['hue']);delta=np.minimum(delta,180-delta)
    matches=mask&(delta<=appearance['hue_tolerance'])&(hsv[:,:,1]>=appearance['saturation_min'])&(hsv[:,:,2]>=appearance['value_min'])
    if np.count_nonzero(matches)<30:raise ValueError('measured_can_body_appearance_not_seen')
    return dict(matching_actual_depth_body_pixels=int(matches.sum()),
        identity_source='continuation_of_verified_grasp_actual_rgbd_surface_and_body_color',new_object_identity_created=False)


def placed_expected(snapshot,plan):
    xy=np.asarray(plan.get('actual_release_xy_map_m',plan['release_xy_map_m']))
    z=float(np.r_[xy,1.]@plan['plane_map'])+plan['height_m']/2
    map_pose(snapshot);tf=snapshot['tracking_transforms']['map_from_base_at_depth']
    return rotation(tf['quaternion']).T@(np.r_[xy,z]-tf['translation'])


def verify_placed(rows,plan,joints,*,cleanup_completed):
    if len(rows)!=3 or not hand_is_open(joints[5]):raise ValueError('three_placed_frames_and_open_hand_required')
    stamps=[r['measurement']['stamp_ns'] for r in rows]
    if any(b-a<150_000_000 for a,b in zip(stamps,stamps[1:])):raise ValueError('distinct_placed_observations_required')
    checks=[region(r['measurement'],r['snapshot'],plan,r['plane']) for r in rows]
    if any(not c['table_edge_safety']['safe'] or abs(c['estimated_bottom_gap_m'])>PLACED_BOTTOM_GAP_M for c in checks):
        raise ValueError('can_not_stably_supported_on_destination')
    centers=np.asarray([c['center_map_m'] for c in checks]);spread=float(np.max(np.linalg.norm(centers-centers.mean(0),axis=1)))
    if spread>PLACED_CENTER_SPREAD_M:raise ValueError('placed_can_not_stable')
    return dict(placement_success_verified=True,object='canned_juice',destination_id=plan['destination_id'],
        checks=checks,center_spread_m=spread,hand_open_verified=True,cleanup_completed=cleanup_completed,
        planned_xy_is_success_gate=False,task_finished_sent=False,full_uninterrupted_transfer_verified=False)

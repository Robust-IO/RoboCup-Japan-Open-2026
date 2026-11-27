"""Can-specific depth tracking and lift evidence. No ROS or robot commands.

Identity is locked by a NEW current-model detection. Near-contact and lifted
measurements must fit observed depth again; predicted positions only select ROI.
"""
import math
import numpy as np

from hand_camera_ray import rotation
from inverse_table_place_plan import OFFSET
from upright_cylinder_geometry import fit_side_circle
from can_approach_plan import LATERAL_ALIGNMENT_TOLERANCE_M

# Retain the tested FORWARD depth only. An old successful can center is not the
# lateral midpoint of the open fingers. At the verified horizontal arm pose,
# the observed palm rotation maps the opening-.65 trigger midpoint to base y
# -0.00005m (gripper-opening-065-v1 and current image-time palm TF).
# Relative to the current palm, never a historical map coordinate or position.
CONTACT_OFFSET_BASE = np.array([.07948608136287283, -.00005, 0.])
CLOSE_TOLERANCE = np.array([.020, LATERAL_ALIGNMENT_TOLERANCE_M, .020])


def upright_trial_evidence(measurement,maximum_tilt_deg=8.):
    """Acceptance for an explicitly selected upright-can experiment only."""
    axis=np.asarray(measurement.get('axis_base_m'),float)
    if (measurement.get('class_name')!='canned_juice'
            or measurement.get('position_source')!='current_cap_plane_and_source_colored_side_depth'
            or axis.shape!=(3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1.)>.001):
        raise ValueError('measured_current_can_axis_required_for_upright_trial')
    tilt=math.degrees(math.acos(float(np.clip(axis[2],-1,1))))
    if not math.isfinite(maximum_tilt_deg) or not 0<maximum_tilt_deg<=15:
        raise ValueError('invalid_upright_trial_acceptance')
    if tilt>maximum_tilt_deg:raise ValueError('upright_trial_can_tipped_stop_before_closure')
    return dict(measured_tilt_deg=tilt,maximum_tilt_deg=maximum_tilt_deg,
        upright_trial_verified=True,motion_authorized=False)


def fresh(snapshot, now_ns, epoch, *, joint_max_age_ns=400_000_000):
    if type(joint_max_age_ns) is not int or joint_max_age_ns not in (400_000_000,600_000_000):
        raise ValueError('invalid_joint_age_limit')
    if not epoch or snapshot.get('tracking_epoch') != epoch:
        raise ValueError('can_task_epoch_mismatch')
    stamps = [snapshot['streams'][k]['stamp_ns'] for k in ('rgb', 'depth')]
    if (any(type(s) is not int or not 0 <= now_ns-s <= 400_000_000 for s in stamps)
            or abs(stamps[0]-stamps[1]) > 30_000_000
            or not -20_000_000 <= now_ns-snapshot['joint_stamp_ns'] <= joint_max_age_ns):
        raise ValueError('fresh_can_rgbd_and_joints_required')


def transform_point(snapshot, point, *, inverse=False):
    tf = snapshot['tracking_transforms']['odom_from_base_at_depth']
    if (tf['stamp_ns'] != snapshot['streams']['depth']['stamp_ns']
            or tf['source_frame'] != 'base_link' or tf['target_frame'] != 'odom'):
        raise ValueError('can_image_time_odom_transform_required')
    r = rotation(tf['quaternion']); t = np.asarray(tf['translation'], float)
    p = np.asarray(point, float)
    if p.shape != (3,) or t.shape != (3,) or not np.isfinite([*p, *t]).all():
        raise ValueError('invalid_can_transform_point')
    return r.T @ (p-t) if inverse else r @ p+t


def grasp_plan(geometry):
    if (geometry.get('class_name') != 'canned_juice' or geometry.get('shape') != 'upright_cylinder'
            or geometry.get('adapter') != 'upright_cylinder_rgbd'):
        raise ValueError('measured_upright_can_required')
    center = np.asarray(geometry['center_m'], float)
    height = geometry['height_m']; radius = geometry['footprint_radius_m']
    if (center.shape != (3,) or not np.isfinite([*center, height, radius]).all()
            or not .035 <= height <= .20 or not .01 <= radius <= .06):
        raise ValueError('invalid_can_dimensions')
    lift = float(center[2]-OFFSET)
    if not .24 <= lift <= .30:
        raise ValueError('can_height_outside_existing_table_preparation_range')
    return dict(class_name='canned_juice',center_base_m=center.tolist(),height_m=height,
        radius_m=radius,grasp_width_m=2*radius,half_height_m=height/2,
        contact_start_lift_m=lift,palm_height_m=float(center[2]),
        bottom_height_m=float(center[2]-height/2),motion_authorized=False)


def anchor(geometry, snapshot, *, task_id):
    plan = grasp_plan(geometry)
    if not task_id or not snapshot.get('tracking_epoch'):
        raise ValueError('can_task_context_required')
    return dict(plan,task_id=task_id,epoch=snapshot['tracking_epoch'],
        center_odom_m=transform_point(snapshot,plan['center_base_m']).tolist(),
        identity_locked=True,identity_source='current_model_detection_and_rgbd',
        source_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        assumption='stationary_until_contact_not_after_lift')


def predicted_before_contact(reference, snapshot, task_id):
    if (reference.get('identity_locked') is not True or reference.get('task_id') != task_id
            or reference.get('epoch') != snapshot.get('tracking_epoch')):
        raise ValueError('can_anchor_context_mismatch')
    return transform_point(snapshot,reference['center_odom_m'],inverse=True)


def contact_offset(extra_seat_m=0.):
    """Extra forward base travel makes the target closer to the current palm."""
    if not math.isfinite(extra_seat_m) or not 0.<=extra_seat_m<=.05:
        raise ValueError('invalid_can_extra_seat_distance')
    return CONTACT_OFFSET_BASE-np.array([extra_seat_m,0.,0.])


def contact_point(snapshot,*,extra_seat_m=0.):
    point = np.asarray(snapshot['palm_base_at_depth'], float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('actual_image_time_palm_required')
    return point+contact_offset(extra_seat_m)


def closure_error(center, snapshot,*,extra_seat_m=0.):
    error = np.asarray(center, float)-contact_point(snapshot,extra_seat_m=extra_seat_m)
    if error.shape != (3,) or not np.isfinite(error).all():
        raise ValueError('invalid_can_contact_error')
    return error


def aligned(center, snapshot,*,extra_seat_m=0.):
    return bool(np.all(np.abs(closure_error(center,snapshot,extra_seat_m=extra_seat_m)) <= CLOSE_TOLERANCE))


def depth_points(depth, snapshot):
    depth = np.asarray(depth, float)
    k = np.asarray(snapshot['rgb_k'], float).reshape(3,3)
    if depth.ndim != 2 or not np.isfinite(k).all() or min(k[0,0],k[1,1]) <= 0:
        raise ValueError('invalid_can_camera_intrinsics')
    tf = snapshot['base_from_rgb']; v,u = np.indices(depth.shape)
    valid = np.isfinite(depth)&(depth>0)&(depth<3.)
    z = depth[valid]
    camera = np.c_[(u[valid]-k[0,2])*z/k[0,0],(v[valid]-k[1,2])*z/k[1,1],z]
    return camera@rotation(tf['quaternion']).T+np.asarray(tf['translation'],float)


def measure_near(depth, snapshot, expected, radius, height, *, rgb=None, appearance=None):
    """Fit visible cap and side in current depth, before OR after a lift.

    Height is the independently measured precontact dimension. After lift, no
    tabletop is treated as the object bottom, and hidden bottom is not measured.
    """
    if rgb is not None or appearance is not None:
        if rgb is None or appearance is None:raise ValueError('both_current_rgb_and_verified_appearance_required')
        from tracked_can_cap_geometry import measure
        return measure(depth,rgb,snapshot,expected,radius,height,appearance)
    expected = np.asarray(expected,float)
    if (expected.shape != (3,) or not np.isfinite([*expected,radius,height]).all()
            or not .01<=radius<=.06 or not .035<=height<=.20):
        raise ValueError('invalid_near_can_reference')
    points = depth_points(depth,snapshot)
    radial = np.linalg.norm(points[:,:2]-expected[:2],axis=1)
    expected_top = expected[2]+height/2
    cap_region = points[(radial<radius+.008)&(np.abs(points[:,2]-expected_top)<.025)]
    if len(cap_region)<40:raise ValueError('current_can_top_not_visible')
    top_z = float(np.quantile(cap_region[:,2],.99))
    top = cap_region[np.abs(cap_region[:,2]-top_z)<.005]
    if len(top)<30 or min(np.ptp(top[:,:2],axis=0))<min(.02,radius):
        raise ValueError('insufficient_current_can_top')
    bottom = top_z-height
    # Once fingers surround a can, the lower/middle surface includes the fingers
    # themselves. Use the visible upper side band below the rim, not those
    # occluded surfaces; still recover circle center/radius from actual points.
    side = points[(radial<radius+.012)&(points[:,2]>bottom+.70*height)
                  &(points[:,2]<bottom+.95*height)]
    # Finger surfaces around the target are not automatically a can. Fit must
    # independently recover the curved can side and a compatible visible cap.
    center_xy,measured_radius,residual,arc = fit_side_circle(side[:,:2],[max(.01,radius-.008),min(.06,radius+.008)])
    center = np.r_[center_xy,top_z-height/2]
    if (np.linalg.norm(center-expected)>.035
            or np.quantile(np.linalg.norm(top[:,:2]-center_xy,axis=1),.95)>measured_radius+.008):
        raise ValueError('current_can_surface_does_not_match_reference')
    return dict(class_name='canned_juice',shape='upright_cylinder',center_m=center.tolist(),
        footprint_radius_m=measured_radius,height_m=height,observed_top_z_m=top_z,
        estimated_bottom_z_m=bottom,side_points=len(side),top_points=len(top),
        cylinder_side_residual_p95_m=residual,observed_side_arc_rad=arc,
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],dimension_source='precontact_rgbd',
        observed_side_height_fraction=[.70,.95],
        hidden_bottom_measured=False,position_source='new_depth_cap_and_side_fit',
        detector_reran=False,identity_confirmed=False,motion_authorized=False)


def lift_follow(before, before_snapshot, after, after_snapshot, opening):
    if before_snapshot.get('tracking_epoch') != after_snapshot.get('tracking_epoch'):
        raise ValueError('can_lift_epoch_mismatch')
    if after['stamp_ns'] <= before['stamp_ns']:
        raise ValueError('can_lift_capture_order')
    p0 = transform_point(before_snapshot,before_snapshot['palm_base_at_depth'])
    p1 = transform_point(after_snapshot,after_snapshot['palm_base_at_depth'])
    c0 = transform_point(before_snapshot,before['center_m'])
    c1 = transform_point(after_snapshot,after['center_m'])
    object_delta = c1-c0; palm_delta = p1-p0
    j0,j1 = before_snapshot['joints'],after_snapshot['joints']
    joint_lift = j1['arm_lift_joint']-j0['arm_lift_joint']
    observed_opening = j1['hand_motor_joint']
    if (not np.isfinite([*object_delta,*palm_delta,joint_lift,opening,observed_opening]).all()
            or not .02<opening<=.62 or not .02<observed_opening<=.62
            or abs(observed_opening-opening)>.03):
        raise ValueError('can_holding_opening_not_preserved')
    if min(object_delta[2],palm_delta[2],joint_lift)<.03:
        raise ValueError('can_did_not_leave_table_with_palm')
    error = float(np.linalg.norm(object_delta-palm_delta))
    if (error>.02 or abs(joint_lift-palm_delta[2])>.02
            or abs(after['footprint_radius_m']-before['footprint_radius_m'])>.008):
        raise ValueError('can_palm_lift_follow_inconsistent')
    return dict(object_delta_odom_m=object_delta.tolist(),palm_delta_odom_m=palm_delta.tolist(),
        joint_lift_m=joint_lift,object_palm_follow_error_m=error,
        center_odom_m=c1.tolist(),center_minus_palm_odom_m=(c1-p1).tolist(),
        observed_opening_rad=observed_opening,stamp_ns=after['stamp_ns'],
        functional_lift_follow_verified=True,placement_verified=False)

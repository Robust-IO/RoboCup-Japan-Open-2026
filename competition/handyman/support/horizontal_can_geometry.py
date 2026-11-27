"""Observe a lying cylinder from current RGBD. No robot control or hidden points.

The body is constrained to be resting on an independently measured table.
Reported center and dimensions are model fits to visible depth, not contact
proof. This module is separate from the verified upright-can adapter.
"""
import math
import numpy as np
from scipy.optimize import least_squares


def horizontal_cylinder_from_points(points, surrounding):
    from object_surface_geometry import support_plane
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 100 or not np.isfinite(points).all():
        raise ValueError('insufficient_horizontal_cylinder_depth')
    plane, support, plane_error = support_plane(surrounding)
    normal = np.r_[-plane[:2], 1.]
    normal /= np.linalg.norm(normal)
    first = np.array([1., 0., plane[0]])
    first /= np.linalg.norm(first)
    second = np.cross(normal, first)
    basis = np.column_stack((first, second, normal))
    origin = np.array([0., 0., plane[2]])
    local = (points - origin) @ basis
    local = local[(local[:, 2] > .004) & (local[:, 2] < .14)]
    if len(local) < 100:
        raise ValueError('horizontal_body_not_above_table')
    xy_origin = np.median(local[:, :2], axis=0)
    local[:, :2] -= xy_origin
    cov = np.cov(local[:, :2].T)
    values, vectors = np.linalg.eigh(cov)
    direction = vectors[:, np.argmax(values)]
    initial_yaw = math.atan2(direction[1], direction[0])
    candidates = []
    for seed in (initial_yaw, initial_yaw + math.pi / 2):
        axis = np.array([math.cos(seed), math.sin(seed)])
        along = local[:, :2] @ axis
        lo, hi = np.quantile(along, [.12, .88])
        body = local[(along >= lo) & (along <= hi)]
        radius = float(np.clip(np.quantile(body[:, 2], .98) / 2, .012, .058))
        def residual(p):
            yaw, side, center_z, r = p
            across = body[:, :2] @ np.array([-math.sin(yaw), math.cos(yaw)]) - side
            return np.hypot(across, body[:, 2] - center_z) - r
        fit = least_squares(residual, [seed, 0., radius, radius],
            bounds=([seed-.5, -.15, .005, .01], [seed+.5, .15, .13, .06]),
            loss='soft_l1', f_scale=.0015, max_nfev=150)
        yaw, side, center_z, radius = fit.x
        error = np.abs(residual(fit.x))
        good = error < .004
        if good.mean() < .85 or good.sum() < 80:
            continue
        p95 = float(np.quantile(error[good], .95))
        if abs(center_z-radius) > .006:
            continue
        axis = np.array([math.cos(yaw), math.sin(yaw)])
        across = np.array([-axis[1], axis[0]])
        arc = np.column_stack((body[good, :2] @ across-side, body[good, 2]-center_z))
        singular = np.linalg.svd(arc-arc.mean(axis=0), compute_uv=False)
        if singular[0] <= 0 or singular[1]/singular[0] < .05:
            continue
        angles = np.sort(np.arctan2(arc[:, 1], arc[:, 0]))
        coverage = float(2*math.pi - np.max(np.diff(np.r_[angles, angles[0]+2*math.pi])))
        if coverage < .65:
            continue
        ends = np.quantile(local[:, :2] @ axis, [.01, .99])
        length = float(ends[1]-ends[0])
        if not max(.06, 2.3*radius) <= length <= .30:
            continue
        center_xy = xy_origin + across*side + axis*float(ends.mean())
        center = origin + basis @ np.r_[center_xy, center_z]
        import cv2
        if cv2.pointPolygonTest(cv2.convexHull(support[:, :2].astype(np.float32)), tuple(map(float, center[:2])), False) < 0:
            continue
        candidates.append(dict(center_base_m=center.tolist(),
            axis_base=(basis[:, :2] @ axis).tolist(), radius_m=float(radius),
            observed_length_m=length, support_plane_base=plane.tolist(),
            support_plane_residual_p95_m=plane_error, side_residual_p95_m=p95,
            side_inlier_fraction=float(good.mean()), observed_arc_rad=coverage,
            source_points=int(len(local)), fit_points=int(good.sum())))
    if not candidates:
        raise ValueError('no_observed_supported_horizontal_cylinder')
    best = min(candidates, key=lambda c: c['side_residual_p95_m'])
    return dict(best, class_name='canned_juice', shape='horizontal_cylinder',
        diameter_m=2*best['radius_m'], motion_authorized=False,
        hidden_surfaces_measured=False, physical_grasp_verified=False,
        dimension_envelope_is_provisional=True)


def estimate(rgb, depth, snapshot, detection):
    from hand_camera_ray import rotation
    from rgbd_pixel_window import pixel_window
    from object_surface_geometry import support_plane
    from upright_cylinder_geometry import select_depth_surface
    if detection.get('name') != 'canned_juice':
        raise ValueError('horizontal_can_class_mismatch')
    depth = np.asarray(depth)
    if depth.ndim != 2 or rgb.shape != (*depth.shape, 3):
        raise ValueError('rgb_depth_shape_mismatch')
    stamps = [snapshot['streams'][k]['stamp_ns'] for k in ('rgb', 'depth')]
    if any(type(s) is not int or s <= 0 for s in stamps) or abs(stamps[0]-stamps[1]) > 30_000_000:
        raise ValueError('unsynchronised_rgbd')
    registration = snapshot['rgb_from_depth']
    if (np.linalg.norm(registration['translation']) > 1e-6
            or not np.allclose(rotation(registration['quaternion']), np.eye(3), atol=1e-6, rtol=0)
            or not np.allclose(snapshot['depth_k'], snapshot['rgb_k'], atol=1e-6, rtol=0)
            or np.any(np.abs(snapshot.get('rgb_d', [0]*5)) > 1e-9)):
        raise ValueError('registered_undistorted_rgbd_required')
    x1, y1, x2, y2 = np.asarray(detection['box'], float)
    h, w = depth.shape
    if not np.isfinite([x1,y1,x2,y2]).all() or not 1 < x1 < x2 < w-1 or not 1 < y1 < y2 < h-1:
        raise ValueError('clipped_or_invalid_horizontal_can_box')
    if min(x2-x1,y2-y1) < 15:
        raise ValueError('horizontal_can_too_small')
    k = np.asarray(snapshot['rgb_k'],float).reshape(3,3)
    if not np.isfinite(k).all() or min(k[0,0],k[1,1]) <= 0:
        raise ValueError('invalid_intrinsics')
    pad = max(25., .6*max(x2-x1,y2-y1))
    window, u, v = pixel_window(depth.shape,x1-pad,y1-pad,x2+pad,y2+pad)
    z = depth[window]
    valid = np.isfinite(z) & (z > 0) & (z < 3)
    points = np.stack(((u-k[0,2])*z/k[0,0], (v-k[1,2])*z/k[1,1], z), axis=-1)
    tf = snapshot['base_from_rgb']
    points = points @ rotation(tf['quaternion']).T + np.asarray(tf['translation'])
    ring = valid & ~((u >= x1-5)&(u < x2+5)&(v >= y1-5)&(v < y2+5))
    plane, _, _ = support_plane(points[ring])
    above = points[:,:,2] - (points[:,:,:2]@plane[:2]+plane[2])
    inside = (u >= x1)&(u < x2)&(v >= y1)&(v < y2)
    foreground = valid & inside & (above > .004)&(above < .14)
    seeds = inside & (u >= x1+.25*(x2-x1))&(u <= x1+.75*(x2-x1))&(v >= y1+.25*(y2-y1))&(v <= y1+.75*(y2-y1))
    selected, segmentation = select_depth_surface(points,foreground,seeds)
    return dict(horizontal_cylinder_from_points(points[selected], points[ring]),
        segmentation=segmentation, frame_id='base_link', stamp_ns=stamps[1],
        confidence=detection.get('confidence'), source='current_rgbd')


def body_appearance(rgb,depth,snapshot,measurement):
    """Bind a body colour only to this measured, operator-confirmed trial can."""
    import cv2
    from held_can_appearance import pixel_points
    p=pixel_points(depth,snapshot);c=np.asarray(measurement['center_base_m']);a=np.asarray(measurement['axis_base'])
    delta=p-c;along=delta@a
    radial=np.linalg.norm(delta-along[:,:,None]*a,axis=2)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask=(np.isfinite(depth)&(depth>0)&(abs(along)<.4*measurement['observed_length_m'])
          &(abs(radial-measurement['radius_m'])<.004)&(hsv[:,:,1]>70)&(hsv[:,:,2]>30))
    hue=hsv[:,:,0][mask]
    if len(hue)<100:raise ValueError('horizontal_can_body_colour_not_observed')
    mode=int(np.bincount(hue,minlength=180).argmax())
    distance=np.minimum(abs(hue.astype(float)-mode),180-abs(hue.astype(float)-mode))
    tolerance=float(max(8.,np.quantile(distance,.95)+3))
    if tolerance>22:raise ValueError('horizontal_can_body_colour_ambiguous')
    return dict(hue=mode,tolerance=tolerance,source_pixels=len(hue),new_identity_detector=False)


def measure_near(rgb,depth,snapshot,expected,axis,reference,appearance):
    """Fit the current coloured body arc. A prior selects ROI, not target depth."""
    import cv2
    from held_can_appearance import pixel_points
    p=pixel_points(depth,snapshot);expected=np.asarray(expected,float);axis=np.asarray(axis,float)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    distance=abs(hsv[:,:,0].astype(float)-appearance['hue']);distance=np.minimum(distance,180-distance)
    delta=p-expected;along=delta@axis;radial=np.linalg.norm(delta-along[:,:,None]*axis,axis=2)
    valid=(np.isfinite(depth)&(depth>0)&(distance<=appearance['tolerance'])&(hsv[:,:,1]>70)&(hsv[:,:,2]>30))
    valid&=(abs(along)<reference['observed_length_m']*.65)&(radial<reference['radius_m']+.045)
    points=p[valid]
    if len(points)<60:raise ValueError('current_horizontal_can_curved_depth_missing')
    # Fit the central body, avoiding cap/rim geometry and adjoining furniture.
    axial=(points-expected)@axis
    body=points[abs(axial)<.35*reference['observed_length_m']]
    if len(body)<60:raise ValueError('current_horizontal_body_belt_missing')
    radius=reference['radius_m'];yaw=math.atan2(axis[1],axis[0]);origin=expected.copy()
    local=body-origin
    def residual(q):
        angle,side,z=q
        across=local[:,:2]@np.array([-math.sin(angle),math.cos(angle)])-side
        return np.hypot(across,local[:,2]-z)-radius
    fit=least_squares(residual,[yaw,0.,0.],bounds=([yaw-.35,-.07,-.07],[yaw+.35,.07,.07]),
        loss='soft_l1',f_scale=.0015,max_nfev=100)
    error=abs(residual(fit.x));good=error<.003
    if good.sum()<60 or good.mean()<.80:raise ValueError('current_horizontal_body_arc_not_consistent')
    yaw,side,z=fit.x;a=np.array([math.cos(yaw),math.sin(yaw),0.]);n=np.array([-a[1],a[0],0.])
    ends=np.quantile((points-origin)@a,[.01,.99]);span=float(ends[1]-ends[0])
    if not .45*reference['observed_length_m']<=span<=1.25*reference['observed_length_m']:
        raise ValueError('current_horizontal_can_axial_extent_not_observed')
    cross=np.c_[local[good]@n-side,local[good,2]-z]
    singular=np.linalg.svd(cross-cross.mean(0),compute_uv=False)
    if singular[-1]/max(singular[0],1e-12)<.04:raise ValueError('current_horizontal_arc_curvature_missing')
    center=origin+n*side+np.array([0,0,z])+a*float(ends.mean())
    return dict(class_name='canned_juice',center_m=center.tolist(),center_base_m=center.tolist(),
        axis_base=a.tolist(),radius_m=radius,observed_length_m=reference['observed_length_m'],
        observed_coloured_span_m=span,source='current_coloured_cylinder_depth',
        current_points=len(points),fit_points=int(good.sum()),side_residual_p95_m=float(np.quantile(error[good],.95)),
        radius_source='this_trial_independent_source_rgbd_fit',hidden_surfaces_measured=False,
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],motion_authorized=False)

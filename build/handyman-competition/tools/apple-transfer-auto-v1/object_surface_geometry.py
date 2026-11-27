"""Measured object proposals, not identity/grasp/contact proof or motion leases.

    Cube fitting assumes an upright approximately cubic opaque object. It uses
    no color mask, Unity object position, or previously estimated object center.
    Hidden faces cannot be certified from one image. Live cube validation is
    deliberately still required before any movement adapter is enabled.
"""
import math
import cv2
import numpy as np

from hand_camera_ray import rotation
from rgbd_pixel_window import pixel_window
from manipulation_profiles import object_profile


def support_plane(points):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<150 or not np.isfinite(points).all():
        raise ValueError('insufficient_local_support')
    bins=np.floor(points[:,2]/.005).astype(int)
    keys,counts=np.unique(bins,return_counts=True)
    selected=points[np.abs(points[:,2]-(keys[np.argmax(counts)]+.5)*.005)<.012]
    if len(selected)<150 or len(selected)/len(points)<.45:raise ValueError('no_dominant_local_support')
    for _ in range(3):
        design=np.c_[selected[:,:2],np.ones(len(selected))]
        coef,_,rank,_=np.linalg.lstsq(design,selected[:,2],rcond=None)
        if rank!=3:raise ValueError('degenerate_local_support')
        selected=selected[np.abs(design@coef-selected[:,2])<=.006]
        if len(selected)<150:raise ValueError('noisy_local_support')
    coef=np.linalg.lstsq(np.c_[selected[:,:2],np.ones(len(selected))],selected[:,2],rcond=None)[0]
    residual=float(np.quantile(np.abs(np.c_[selected[:,:2],np.ones(len(selected))]@coef-selected[:,2]),.95))
    if residual>.004 or np.linalg.norm(coef[:2])>math.tan(math.radians(10)):
        raise ValueError('nonhorizontal_or_noisy_support')
    if min(np.ptp(selected[:,:2],axis=0))<.04:raise ValueError('insufficient_support_extent')
    return coef,selected,residual


def cube_from_points(points, surrounding, profile):
    """Observation-space model fit; no claim that hidden surfaces were measured."""
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<80 or not np.isfinite(points).all():
        raise ValueError('insufficient_cube_surface')
    coef,plane_points,plane_residual=support_plane(surrounding)
    heights=points[:,2]-np.c_[points[:,:2],np.ones(len(points))]@coef
    points=points[(heights>.004)&(heights<profile['side_range_m'][1]+.02)]
    if len(points)<80:raise ValueError('insufficient_above_support_surface')
    heights=points[:,2]-np.c_[points[:,:2],np.ones(len(points))]@coef
    height=float(np.quantile(heights,.99))
    top=points[np.abs(heights-height)<.005]
    if len(top)<30:raise ValueError('cube_top_not_observed')
    rect=cv2.minAreaRect(points[:,:2].astype(np.float32))
    center_xy=np.asarray(rect[0],float);sides=np.asarray(rect[1],float)
    lo,hi=profile['side_range_m']
    if (not np.isfinite([*center_xy,*sides,height]).all()
            or not lo<=height<=hi or np.any(sides<lo) or np.any(sides>hi)
            or np.any(sides/height<.65) or np.any(sides/height>1.35)):
        raise ValueError('not_an_observable_upright_cube')
    if cv2.pointPolygonTest(cv2.convexHull(plane_points[:,:2].astype(np.float32)),tuple(center_xy),False)<0:
        raise ValueError('cube_outside_observed_support')
    angle=math.radians(rect[2]);r=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    projected=(top[:,:2]-center_xy)@r
    if np.any(np.ptp(projected,axis=0)<sides*.55):raise ValueError('cube_top_coverage_incomplete')
    bottom=float(np.r_[center_xy,1.]@coef)
    return dict(shape='upright_cube',center_m=[*center_xy,bottom+height/2],height_m=height,
                footprint_size_m=sides.tolist(),footprint_radius_m=float(np.linalg.norm(sides)/2),
                estimated_grasp_width_m=float(min(sides)),footprint_yaw_rad=angle,
                support_plane_base=coef.tolist(),support_height_m=bottom,
                support_plane_residual_p95_m=plane_residual,surface_points=len(points),top_points=len(top),
                center_semantics='support_constrained_upright_cube_model_fit',
                hidden_surfaces_measured=False,geometry_fit_is_not_total_uncertainty=True,
                dimension_envelope_is_provisional=True,motion_authorized=False,
                identity_confirmed=False,grasp_center_verified=False,
                limitation='Single-view upright-cube model proposal; not an amodal shape, confirmed contact, calibrated grasp center or successful grasp.')


def estimate(rgb,depth,snapshot,detection,name):
    profile=object_profile(name)
    if detection.get('name')!=name:raise ValueError('target_class_mismatch')
    confidence=detection.get('confidence')
    if type(confidence) not in (int,float) or not math.isfinite(confidence) or not .1<=confidence<=1:
        raise ValueError('invalid_geometry_proposal_score')
    if profile['geometry']=='legacy_apple_red_sphere':
        from apple_rgbd_candidate import estimate as apple_estimate
        # Preserve the original numeric measurement and all original flags.
        return dict(adapter=profile['geometry'],legacy_candidate=apple_estimate(rgb,depth,snapshot,detection,geometry_proposal_only=True),
                    class_name=name,motion_authorized=False)
    if profile['geometry']=='penguin_two_surface_rgbd':
        from penguin_geometry import estimate_source
        return estimate_source(rgb,depth,snapshot,detection)
    if profile['geometry']=='round_shaker_rgbd':
        from pepper_geometry import estimate_source
        return estimate_source(rgb,depth,snapshot,detection)
    if rgb.ndim!=3 or rgb.shape[2]!=3 or depth.ndim!=2 or rgb.shape[:2]!=depth.shape:
        raise ValueError('rgb_depth_shape_mismatch')
    stamps=[snapshot['streams'][key]['stamp_ns'] for key in ('rgb','depth')]
    if any(type(s) is not int or s<=0 for s in stamps) or abs(stamps[0]-stamps[1])>30_000_000:
        raise ValueError('unsynchronised_rgbd')
    box=np.asarray(detection['box'],float);h,w=depth.shape
    if box.shape!=(4,) or not np.isfinite(box).all():raise ValueError('invalid_detection_box')
    x1,y1,x2,y2=box
    if not 1<x1<x2<w-1 or not 1<y1<y2<h-1:raise ValueError('target_clipped_or_invalid_box')
    if min(x2-x1,y2-y1)<15:raise ValueError('target_too_small_for_object_geometry')
    k=np.asarray(snapshot['rgb_k'],float).reshape(3,3)
    if not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0:raise ValueError('invalid_intrinsics')
    if np.any(np.abs(np.asarray(snapshot.get('rgb_d',[0]*5)))>1e-9):raise ValueError('undistorted_rgbd_required')
    registration=snapshot.get('rgb_from_depth')
    if (not isinstance(registration,dict)
            or np.asarray(registration.get('translation')).shape!=(3,)
            or not np.isfinite(registration['translation']).all()
            or np.linalg.norm(registration['translation'])>1e-6
            or np.linalg.norm(rotation(registration['quaternion'])-np.eye(3))>1e-6
            or 'depth_k' in snapshot and not np.allclose(snapshot['depth_k'],snapshot['rgb_k'],atol=1e-6,rtol=0)):
        raise ValueError('registered_rgb_depth_required')
    pad=max(25.,.6*max(x2-x1,y2-y1))
    window,u,v=pixel_window(depth.shape,x1-pad,y1-pad,x2+pad,y2+pad)
    z=depth[window]
    valid=np.isfinite(z)&(z>0)&(z<3)
    points=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),axis=-1)
    tf=snapshot['base_from_rgb'];points=points@rotation(tf['quaternion']).T+np.asarray(tf['translation'],float)
    if not np.isfinite(points[valid]).all():raise ValueError('invalid_image_time_transform')
    inside=(u>=x1+1)&(u<x2-1)&(v>=y1+1)&(v<y2-1)
    if np.count_nonzero(valid&inside)<80:raise ValueError('no_valid_target_depth')
    ring=valid&~((u>=x1-5)&(u<x2+5)&(v>=y1-5)&(v<y2+5))
    coef,_,_=support_plane(points[ring])
    heights=points[:,:,2]-(points[:,:,:2]@coef[:2]+coef[2])
    max_height=profile['height_range_m'][1] if profile['geometry']=='upright_cylinder_rgbd' else profile['side_range_m'][1]
    foreground=valid&inside&(heights>.004)&(heights<max_height+.02)
    # Spatial components, not a red-pixel mask. Multiple unresolved surfaces are
    # reported as ambiguous rather than selecting the largest background object.
    if profile['geometry']=='upright_cylinder_rgbd':
        from upright_cylinder_geometry import cylinder_from_points,select_depth_surface
        seeds=(u>=x1+.3*(x2-x1))&(u<x1+.7*(x2-x1))&(v>=y1+.25*(y2-y1))&(v<y1+.75*(y2-y1))
        selected,segmentation=select_depth_surface(points,foreground,seeds)
        result=cylinder_from_points(points[selected],points[ring],profile)
        result['surface_segmentation']=segmentation
    else:
        _,labels,stats,_=cv2.connectedComponentsWithStats(foreground.astype(np.uint8),connectivity=8)
        components=np.flatnonzero(stats[1:,cv2.CC_STAT_AREA]>=80)+1
        if len(components)!=1:raise ValueError('ambiguous_or_missing_object_surface')
        result=cube_from_points(points[labels==components[0]],points[ring],profile)
    return dict(result,adapter=profile['geometry'],class_name=name,source='rgbd',frame_id='base_link',
                stamp_ns=stamps[1],confidence=confidence,depth_valid=True,
                support_consistent=True,room_confirmed=False,ready_for_motion=False)

"""Held-penguin evidence. Image projection is a ROI, never measured object XYZ."""
import math
import cv2
import numpy as np
from hand_camera_ray import rotation
from penguin_geometry import NAME,measure_near,validate_reference,colour_mask,near_window,fit_sphere


def current_cap_depth(rgb,depth,snapshot,palm,reference):
    """Continue an already RGBD-verified grasp when its body is behind the wrist.

    Returns a measured cap only; no hidden-body XYZ or new identity is created.
    """
    body,cap=expected_centers(palm,reference)
    opening=snapshot['joints']['hand_motor_joint'];holding=reference['holding_opening_rad']
    if not .02<opening<=.62 or abs(opening-holding)>.03:raise ValueError('penguin_current_grip_not_preserved')
    if rgb.shape!=(*depth.shape,3):raise ValueError('penguin_rgbd_shape_mismatch')
    model=reference['model'];radius=model['cap_radius_m']
    window,u,v=near_window(depth.shape,snapshot,[cap],[radius+.012])
    if u.size==0:raise ValueError('penguin_cap_not_observed')
    z=depth[window];k=np.asarray(snapshot['rgb_k'],float).reshape(3,3);tf=snapshot['base_from_rgb']
    camera=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),axis=-1)
    points=camera@rotation(tf['quaternion']).T+tf['translation']
    axis=cap-body;axis=axis/np.linalg.norm(axis)
    mask=np.isfinite(z)&(z>0)&(z<3.)&colour_mask(rgb[window],model['appearance']['cap'])
    mask&=np.linalg.norm(points-cap,axis=-1)<radius+.012
    mask&=(points-cap)@axis>-.003
    center,actual_radius,fit=fit_sphere(points[mask],[radius-.003,radius+.003])
    error=float(np.linalg.norm(center-cap))
    if error>.02:raise ValueError('penguin_cap_not_at_verified_palm_offset')
    return dict(class_name=NAME,current_cap_depth_verified=True,current_depth_surface_verified=False,
        full_body_measurement_available=False,measurement=None,new_target_identity_created=False,
        cap_depth_measurement=dict(center_base_m=center.tolist(),radius_m=actual_radius,fit=fit,
            stamp_ns=snapshot['streams']['depth']['stamp_ns']),palm_cap_offset_error_m=error,
        opening_rad=opening,source_identity='continuation_of_this_rgbd_verified_penguin_grasp',motion_authorized=False)


def palm_points(body,cap,palm):
    r=rotation(palm['quaternion']);p=np.asarray(palm['translation'])
    return (r.T@(np.asarray(body)-p)).tolist(),(r.T@(np.asarray(cap)-p)).tolist()


def expected_centers(palm,reference):
    if reference.get('class_name')!=NAME or reference.get('grasp_lift_verified') is not True:
        raise ValueError('verified_toy_penguin_hold_required')
    validate_reference(reference['model']);r=rotation(palm['quaternion']);p=np.asarray(palm['translation'])
    return p+r@reference['body_palm_m'],p+r@reference['cap_palm_m']


def current_depth(rgb,depth,snapshot,palm,reference):
    body,cap=expected_centers(palm,reference)
    holding=reference['holding_opening_rad'];opening=snapshot['joints']['hand_motor_joint']
    if not .02<opening<=.62 or abs(opening-holding)>.03:raise ValueError('penguin_current_grip_not_preserved')
    measured=measure_near(rgb,depth,snapshot,body,cap,reference['model'])
    error=max(float(np.linalg.norm(np.asarray(measured['center_m'])-body)),
        float(np.linalg.norm(np.asarray(measured['cap_center_m'])-cap)))
    if error>.02:raise ValueError('penguin_no_longer_at_verified_palm_offset')
    return dict(class_name=NAME,current_depth_surface_verified=True,current_rgb_appearance_verified=False,
        full_body_measurement_available=measured.get('current_body_depth_measured',True),
        current_head_depth_verified=measured.get('current_head_depth_measured',True),
        measurement=measured,palm_offset_error_m=error,opening_rad=opening,
        grip_feedback_alone_is_not_proof=True,motion_authorized=False)


def rgb_continuation(rgb,depth,snapshot,palm,reference):
    from canned_carry_geometry import tucked_holding
    body,cap=expected_centers(palm,reference);model=reference['model'];holding=reference['holding_opening_rad']
    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    if not tucked_holding([snapshot['joints'][n] for n in names],holding):
        raise ValueError('penguin_rgb_continuation_requires_tucked_actual_grip')
    if rgb.shape[:2]!=depth.shape or np.count_nonzero(np.isfinite(depth)&(depth>0))<1000:
        raise ValueError('penguin_scene_depth_not_observed')
    tf=snapshot['base_from_rgb'];r=rotation(tf['quaternion']);origin=np.asarray(tf['translation'])
    optical=(np.asarray([body,cap])-origin)@r;k=np.asarray(snapshot['rgb_k']).reshape(3,3)
    if min(optical[:,2])<=.05:raise ValueError('penguin_held_projection_behind_camera')
    pixels=optical@k.T;pixels=pixels[:,:2]/pixels[:,2,None]
    radii=np.array([model['body_radius_m'],model['cap_radius_m']]);pixel_radii=radii*max(k[0,0],k[1,1])/optical[:,2]
    if min(optical[:,2])>=.415:raise ValueError('penguin_use_depth_when_surface_is_not_near_clipped')
    selected=[]
    for index,name in enumerate(('body','cap')):
        center=pixels[index];rad=pixel_radii[index];u,v=np.rint(center).astype(int)
        if not 0<=u<depth.shape[1] or not 0<=v<depth.shape[0]:raise ValueError('penguin_held_projection_outside_image')
        tile=depth[max(0,v-7):v+8,max(0,u-7):u+8]
        if np.mean(~np.isfinite(tile)|(tile<=0))<.5:raise ValueError('penguin_actual_depth_available_use_it')
        mask=colour_mask(rgb,model['appearance'][name]).astype(np.uint8)
        yy,xx=np.indices(depth.shape);mask[((xx-center[0])**2+(yy-center[1])**2)>(rad*1.7+8)**2]=0
        _,labels,stats,centers=cv2.connectedComponentsWithStats(mask,connectivity=8)
        candidates=[]
        for label,st in enumerate(stats[1:],1):
            if st[4]<max(30,.12*math.pi*rad**2):continue
            if not .5*rad<=st[2]<=3.5*rad or not .35*rad<=st[3]<=3.5*rad:continue
            error=float(np.linalg.norm(centers[label]-center))
            if error>max(8,.85*rad):continue
            available=depth[(labels==label)&np.isfinite(depth)&(depth>0)]
            if len(available)>=30 and np.median(available)>optical[index,2]+radii[index]+.08:continue
            candidates.append((label,st,centers[label],error))
        if len(candidates)!=1:raise ValueError('penguin_unique_current_'+name+'_appearance_not_seen')
        label,st,center,error=candidates[0]
        selected.append(dict(name=name,center_pixel=center.tolist(),box=st[:4].tolist(),pixels=int(st[4]),
            expected_projection_pixel=pixels[index].tolist(),projection_error_px=error))
    observed=np.asarray(selected[1]['center_pixel'])-selected[0]['center_pixel']
    if np.linalg.norm(observed-(pixels[1]-pixels[0]))>max(10,.5*pixel_radii.sum()):
        raise ValueError('penguin_current_body_cap_image_pair_inconsistent')
    return dict(class_name=NAME,current_rgb_appearance_verified=True,current_depth_surface_verified=False,
        measurement=None,measured_3d_object_position_available=False,new_target_identity_created=False,
        source_identity='continuation_of_this_rgbd_verified_penguin_grasp',
        opening_rad=snapshot['joints']['hand_motor_joint'],body=selected[0],cap=selected[1],
        near_depth_predictions_only=True,motion_authorized=False)

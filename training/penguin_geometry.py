"""Toy-penguin body/head measured from registered RGBD, grasp at its head.

The wide body is retained for support/extent tracking. The small dark head is
an independently observed grasp surface. No Unity object coordinates are used.
"""
import math
import cv2
import numpy as np
from hand_camera_ray import rotation
from rgbd_pixel_window import pixel_window
from pepper_geometry import fit_sphere,colour_model,colour_mask,near_window,source_support_plane,surface_extent,lift_follow as full_surface_lift_follow

NAME='toy_penguin'
SHAPE='rounded_penguin_body_and_head'
BODY_RANGE=(.03,.07)
HEAD_RANGE=(.015,.035)
HEIGHT_RANGE=(.08,.17)

def connected_object_surface(points,foreground,body,body_radius,head,head_radius):
    """Keep depth-connected pixels seeded by the two actually fitted surfaces.

    A detection rectangle also contains background pixels. Height above the
    table alone must not turn a distant pot into part of the penguin's extent.
    Adjacency is in both the image and measured 3D space, with no radius clipping.
    """
    from collections import deque
    seeds=foreground&((abs(np.linalg.norm(points-body,axis=-1)-body_radius)<.003)
                     |(abs(np.linalg.norm(points-head,axis=-1)-head_radius)<.003))
    selected=seeds.copy();queue=deque(map(tuple,np.argwhere(seeds)))
    height,width=foreground.shape
    while queue:
        y,x=queue.popleft();p=points[y,x]
        for dy,dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
            yy,xx=y+dy,x+dx
            if not (0<=yy<height and 0<=xx<width) or selected[yy,xx] or not foreground[yy,xx]:continue
            delta=points[yy,xx]-p
            if float(delta@delta)<=.012**2:
                selected[yy,xx]=True;queue.append((yy,xx))
    return selected

def fit_observed_radius(points,radius,expected):
    """Track a partly occluded rigid surface with its source-measured size.

    The centre is fitted to CURRENT points. The previous centre initializes the
    optimizer only; no projected or hidden surface points enter this fit.
    """
    from scipy.optimize import least_squares
    p=np.asarray(points,float);expected=np.asarray(expected,float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<60 or not np.isfinite(p).all():
        raise ValueError('penguin_current_curved_surface_not_observed')
    singular=np.linalg.svd(p-p.mean(0),compute_uv=False)
    if singular[-1]<.001 or singular[-1]/singular[0]<.025:
        raise ValueError('penguin_current_surface_curvature_not_observed')
    selected=p;center=expected.copy()
    for _ in range(4):
        solved=least_squares(lambda c:np.linalg.norm(selected-c,axis=1)-radius,
                             center,loss='soft_l1',f_scale=.001,max_nfev=100)
        if not solved.success:raise ValueError('penguin_current_surface_fit_failed')
        center=solved.x;error=abs(np.linalg.norm(p-center,axis=1)-radius)
        selected=p[error<.002]
        if len(selected)<60:raise ValueError('penguin_current_surface_inliers_not_observed')
    if len(selected)<.65*len(p):raise ValueError('penguin_current_surface_outlier_fraction')
    curved=np.linalg.svd(selected-selected.mean(0),compute_uv=False)
    if curved[-1]<.001 or curved[-1]/curved[0]<.025:
        raise ValueError('penguin_current_surface_curvature_not_observed')
    residual=float(np.quantile(abs(np.linalg.norm(selected-center,axis=1)-radius),.95))
    if residual>.0025:raise ValueError('penguin_current_surface_residual')
    return center,radius,dict(points=len(p),inliers=len(selected),residual_p95_m=residual,
        all_candidate_residual_p95_m=float(np.quantile(error,.95)),outlier_fraction=1-len(selected)/len(p),
        radius_source='this_target_source_rgbd',center_source='current_visible_depth_fit',hidden_points_used=False)


def estimate_source(rgb,depth,snapshot,detection,*,measured_support=None,allow_clipped_seed=False):
    from object_surface_geometry import support_plane
    if detection.get('name')!=NAME or not .1<=detection.get('confidence',-1)<=1:
        raise ValueError('toy_penguin_detection_required')
    if rgb.shape!=(*depth.shape,3):raise ValueError('penguin_rgbd_shape_mismatch')
    stamps=[snapshot['streams'][key]['stamp_ns'] for key in ('rgb','depth')]
    if any(type(s) is not int or s<=0 for s in stamps) or abs(stamps[0]-stamps[1])>30_000_000:
        raise ValueError('penguin_synchronised_rgbd_required')
    k=np.asarray(snapshot['rgb_k'],float).reshape(3,3);reg=snapshot.get('rgb_from_depth',{})
    if (not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0
            or np.any(np.abs(snapshot.get('rgb_d',[0]*5))>1e-9)
            or np.asarray(reg.get('translation')).shape!=(3,) or not np.isfinite(reg['translation']).all()
            or np.linalg.norm(reg['translation'])>1e-6
            or not np.allclose(rotation(reg['quaternion']),np.eye(3),atol=1e-6,rtol=0)
            or not np.allclose(snapshot.get('depth_k',snapshot['rgb_k']),snapshot['rgb_k'],atol=1e-6,rtol=0)):
        raise ValueError('penguin_registered_undistorted_depth_required')
    box=np.asarray(detection['box'],float);h,w=depth.shape
    if box.shape!=(4,) or not np.isfinite(box).all():raise ValueError('penguin_invalid_box')
    x1,y1,x2,y2=box
    box_valid=(0<=x1<x2<=w and 0<=y1<y2<=h) if allow_clipped_seed else (1<x1<x2<w-1 and 1<y1<y2<h-1)
    if not box_valid or min(x2-x1,y2-y1)<25:
        raise ValueError('penguin_clipped_or_small_detection')
    pad=max(30.,.65*max(x2-x1,y2-y1));window,u,v=pixel_window(depth.shape,x1-pad,y1-pad,x2+pad,y2+pad)
    z=depth[window];valid=np.isfinite(z)&(z>0)&(z<3.)
    camera=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),axis=-1)
    tf=snapshot['base_from_rgb'];points=camera@rotation(tf['quaternion']).T+tf['translation']
    inside=(u>x1)&(u<x2)&(v>y1)&(v<y2)
    ring=valid&~((u>x1-8)&(u<x2+8)&(v>y1-8)&(v<y2+8))
    if measured_support is None:plane,_,residual=support_plane(points[ring])
    else:
        plane=source_support_plane(measured_support,snapshot);local=points[ring]
        inliers=local[abs(local[:,:2]@plane[:2]+plane[2]-local[:,2])<.004]
        if len(inliers)<150 or min(np.ptp(inliers[:,:2],axis=0))<.04:raise ValueError('penguin_local_table_not_observed')
        residual=float(np.quantile(abs(inliers[:,:2]@plane[:2]+plane[2]-inliers[:,2]),.95))
    heights=points[:,:,2]-(points[:,:,:2]@plane[:2]+plane[2])
    foreground=valid&inside&(heights>.004)&(heights<HEIGHT_RANGE[1]+.02)
    if np.count_nonzero(foreground)<150:raise ValueError('penguin_surface_not_observed')
    height=float(np.quantile(heights[foreground],.99))
    if not HEIGHT_RANGE[0]<=height<=HEIGHT_RANGE[1]:raise ValueError('penguin_height_outside_envelope')
    local_rgb=rgb[window];hsv=cv2.cvtColor(local_rgb,cv2.COLOR_RGB2HSV)
    body_mask=foreground&(heights>.18*height)&(heights<.57*height)&(hsv[:,:,2]>90)
    head_mask=foreground&(heights>.73*height)&(heights<1.01*height)&(hsv[:,:,2]<110)
    body,r,bfit=fit_sphere(points[body_mask],BODY_RANGE)
    head,hr,hfit=fit_sphere(points[head_mask],HEAD_RANGE)
    distance=float(np.linalg.norm(head-body));axis=(head-body)/distance
    support=float(np.r_[body[:2],1.]@plane);above=float(body[2]-support)
    head_above=float(head[2]-(np.r_[head[:2],1.]@plane))
    if not .04<distance<.095 or not .25<hr/r<.8 or axis[2]<math.cos(math.radians(12)) or not .015<above<r:
        raise ValueError('penguin_body_head_geometry_inconsistent')
    foreground=connected_object_surface(points,foreground,body,r,head,hr)
    # Wings are below the head. Measure the dark lateral surface actually seen,
    # rather than using hidden mesh dimensions to authorize finger clearance.
    lateral=np.linalg.norm(points[:,:,:2]-body[:2],axis=-1)>hr*1.4
    wing=foreground&lateral&(hsv[:,:,2]<110)&(heights<head_above-hr*.7)
    if np.count_nonzero(wing)<30:raise ValueError('penguin_wing_clearance_not_observed')
    wing_top=float(np.quantile(heights[wing],.99))
    grasp_offset=max(0.,wing_top+.0175+.004-head_above)
    if grasp_offset>min(.006,hr*.25):raise ValueError('penguin_head_fingers_would_intersect_wings')
    grasp=head+grasp_offset*axis
    appearance=dict(version=1,class_name=NAME,source='actual_verified_toy_penguin_rgbd',
        body=colour_model(local_rgb,body_mask),cap=colour_model(local_rgb,head_mask),new_target_identification_supported=False)
    # Visible lateral extent includes wings in the conservative support radius.
    extent=float(np.quantile(np.linalg.norm(points[foreground,:2]-body[:2],axis=-1),.995))
    return dict(class_name=NAME,shape=SHAPE,adapter='penguin_two_surface_rgbd',center_m=body.tolist(),cap_center_m=head.tolist(),
        body_radius_m=r,cap_radius_m=hr,body_fit=bfit,cap_fit=hfit,footprint_radius_m=max(r,extent),height_m=height,
        body_above_support_m=above,body_cap_distance_m=distance,axis_base_m=axis.tolist(),support_plane_base=plane.tolist(),
        support_height_m=support,support_plane_residual_p95_m=residual,head_above_support_m=head_above,
        observed_wing_top_above_support_m=wing_top,observed_horizontal_extent_m=max(r,extent),
        appearance=appearance,stamp_ns=stamps[1],confidence=detection['confidence'],source='rgbd',frame_id='base_link',
        grasp_center_m=grasp.tolist(),head_grasp_axial_offset_m=grasp_offset,grasp_center_semantics='observed_head_with_measured_finger_wing_clearance_offset',estimated_grasp_width_m=2*hr,
        depth_valid=True,support_consistent=True,room_confirmed=False,ready_for_motion=False,partial_seed_only=allow_clipped_seed,
        hidden_surfaces_measured=False,motion_authorized=False)


def validate_reference(ref):
    if (ref.get('class_name')!=NAME or ref.get('shape')!=SHAPE or ref.get('appearance',{}).get('source')!='actual_verified_toy_penguin_rgbd'
            or not BODY_RANGE[0]<=ref['body_radius_m']<=BODY_RANGE[1] or not HEAD_RANGE[0]<=ref['cap_radius_m']<=HEAD_RANGE[1]
            or not HEIGHT_RANGE[0]<=ref['height_m']<=HEIGHT_RANGE[1] or not .04<ref['body_cap_distance_m']<.095
            or not 0<=ref.get('head_grasp_axial_offset_m',float('nan'))<=min(.006,ref['cap_radius_m']*.25)
            or not ref['body_radius_m']<=ref.get('observed_horizontal_extent_m',float('nan'))<=.09):
        raise ValueError('verified_toy_penguin_reference_required')

def foreground_occlusion(depth,snapshot,body,radius):
    tf=snapshot['base_from_rgb'];p=rotation(tf['quaternion']).T@(np.asarray(body)-tf['translation'])
    if p[2]<=radius:raise ValueError('penguin_body_occlusion_not_observed')
    k=np.asarray(snapshot['rgb_k']).reshape(3,3);u,v=np.rint((k@p)[:2]/p[2]).astype(int)
    if not 3<=u<depth.shape[1]-3 or not 3<=v<depth.shape[0]-3:
        raise ValueError('penguin_body_occlusion_not_observed')
    tile=depth[v-3:v+4,u-3:u+4];values=tile[np.isfinite(tile)&(tile>0)]
    if len(values)<20 or float(np.median(values))>=p[2]-radius-.01:
        raise ValueError('penguin_body_occlusion_not_observed')
    return dict(foreground_occlusion_observed=True,center_pixel=[int(u),int(v)],
                foreground_depth_m=float(np.median(values)),predicted_body_front_depth_m=float(p[2]-radius))


def measure_near(rgb,depth,snapshot,expected_body,expected_cap,ref,*,open_pregrasp=False):
    validate_reference(ref)
    if open_pregrasp:
        from task_outcome_policy import hand_is_open
        if not hand_is_open(snapshot.get('joints',{}).get('hand_motor_joint',-1)):
            raise ValueError('open_pregrasp_requires_actual_open_hand')
    association_limit=.06 if open_pregrasp else .025
    padding=.015
    b,c=np.asarray(expected_body,float),np.asarray(expected_cap,float)
    if b.shape!=(3,) or c.shape!=(3,) or not np.isfinite([*b,*c]).all():raise ValueError('invalid_penguin_roi_prediction')
    axis=c-b;length=float(np.linalg.norm(axis))
    if not .035<length<.105:raise ValueError('invalid_penguin_body_head_prediction')
    axis/=length
    window,u,v=near_window(depth.shape,snapshot,[b,c],[ref['body_radius_m']+padding,ref['cap_radius_m']+padding])
    if u.size==0 or rgb.shape!=(*depth.shape,3):raise ValueError('penguin_current_surface_not_observed')
    z=depth[window];k=np.asarray(snapshot['rgb_k']).reshape(3,3);tf=snapshot['base_from_rgb']
    camera=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),-1)
    points=camera@rotation(tf['quaternion']).T+tf['translation'];valid=np.isfinite(z)&(z>0)&(z<3.)
    axial=(points-b)@axis
    def fit_part(name,center,radius):
        selected=valid&colour_mask(rgb[window],ref['appearance'][name])
        selected&=np.linalg.norm(points-center,axis=-1)<radius+padding
        if name=='body':selected&=(axial>.18*ref['height_m']-ref['body_above_support_m'])&(axial<.57*ref['height_m']-ref['body_above_support_m'])
        else:selected&=(points-c)@axis>-.005
        observed=points[selected]
        try:measured,r,fit=fit_sphere(observed,[radius-.003,radius+.003])
        except ValueError as exc:
            if str(exc) not in ('pepper_radius_outside_observed_envelope','pepper_surface_residual'):raise
            measured,r,fit=fit_observed_radius(observed,radius,center)
        if np.linalg.norm(measured-center)>association_limit:raise ValueError('penguin_current_surface_outside_association')
        return measured,r,fit
    head,hr,hfit=fit_part('cap',c,ref['cap_radius_m'])
    inferred=False;occlusion=None
    try:body,r,bfit=fit_part('body',b,ref['body_radius_m'])
    except ValueError as exc:
        # A previously identified connected rigid object may be hidden by the
        # wrist. Only its head remains a current independent depth observation.
        body=head-axis*ref['body_cap_distance_m'];r=ref['body_radius_m']
        occlusion=foreground_occlusion(depth,snapshot,body,r)
        inferred=True;bfit=dict(points=0,current_body_surface_measured=False,reason=str(exc),
            center_source='source_rigid_shape_registered_to_current_head_depth')
    distance=float(np.linalg.norm(head-body))
    if abs(distance-ref['body_cap_distance_m'])>.007:raise ValueError('penguin_body_head_pair_inconsistent')
    shape=surface_extent(body,head,r,hr,ref['body_above_support_m'])
    shape['horizontal_support_radius_m']=max(shape['horizontal_support_radius_m'],ref['observed_horizontal_extent_m'])
    return dict(class_name=NAME,shape=SHAPE,center_m=body.tolist(),cap_center_m=head.tolist(),grasp_center_m=(head+ref['head_grasp_axial_offset_m']*(head-body)/distance).tolist(),
        body_radius_m=r,cap_radius_m=hr,body_fit=bfit,cap_fit=hfit,footprint_radius_m=shape['horizontal_support_radius_m'],
        height_m=ref['height_m'],body_above_support_m=ref['body_above_support_m'],body_cap_distance_m=distance,**shape,
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],hidden_surfaces_measured=False,predicted_center_returned=inferred,
        current_body_depth_measured=not inferred,current_head_depth_measured=True,body_occlusion=occlusion,
        position_source='current_head_depth_and_rigid_body_estimate' if inferred else 'current_body_and_head_sphere_depth_fits',open_pregrasp=open_pregrasp,
        association_limit_m=association_limit,motion_authorized=False)

def lift_follow(before,bs,after,ass,opening):
    if before.get('current_body_depth_measured',True) and after.get('current_body_depth_measured',True):
        return dict(full_surface_lift_follow(before,bs,after,ass,opening),
                    measured_part='body_and_head',body_motion_independently_measured=True)
    from canned_grasp_geometry import transform_point
    if (bs.get('tracking_epoch')!=ass.get('tracking_epoch') or after['stamp_ns']<=before['stamp_ns']
            or not before.get('current_head_depth_measured') or not after.get('current_head_depth_measured')):
        raise ValueError('penguin_current_head_lift_evidence_required')
    palm=transform_point(ass,ass['palm_base_at_depth'])-transform_point(bs,bs['palm_base_at_depth'])
    head=transform_point(ass,after['cap_center_m'])-transform_point(bs,before['cap_center_m'])
    joint=ass['joints']['arm_lift_joint']-bs['joints']['arm_lift_joint']
    error=float(np.linalg.norm(head-palm))
    if (not .02<opening<=.62 or abs(ass['joints']['hand_motor_joint']-opening)>.03
            or min(head[2],palm[2],joint)<.03 or error>.02):
        raise ValueError('penguin_head_did_not_follow_palm_off_table')
    return dict(stamp_ns=after['stamp_ns'],grasp_lift_verified=True,placement_verified=False,
        measured_part='head',body_motion_independently_measured=False,object_palm_follow_error_m=error,
        object_delta_odom_m=head.tolist(),cap_delta_odom_m=head.tolist(),palm_delta_odom_m=palm.tolist(),
        body_relation_source='previously_observed_connected_rigid_penguin')

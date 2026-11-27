"""Ground-pepper RGBD geometry: wide rounded body and smaller rounded cap.

Both surfaces are fitted from observed nonzero depth. Their distinct radii and
centers keep the narrow neck out of the grasp. Saved dimensions select a local
ROI during tracking; they never substitute for a newly observed surface.
"""
import math
import cv2
import numpy as np

from hand_camera_ray import rotation
from rgbd_pixel_window import pixel_window

NAME='ground_pepper'
BODY_RANGE=(.015,.045)
CAP_RANGE=(.008,.030)
HEIGHT_RANGE=(.04,.12)


def fit_sphere(points,radius_range,*,minimum=60):
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<minimum or not np.isfinite(p).all():
        raise ValueError('pepper_surface_not_observed')
    origin=p.mean(0);q=p-origin;singular=np.linalg.svd(q,compute_uv=False)
    if singular[-1]<.001 or singular[-1]/singular[0]<.025:
        raise ValueError('pepper_curvature_not_observed')
    selected=q
    for _ in range(4):
        design=np.c_[2*selected,np.ones(len(selected))]
        value,_,rank,_=np.linalg.lstsq(design,np.sum(selected*selected,axis=1),rcond=None)
        squared=value[3]+value[:3]@value[:3]
        if rank!=4 or not math.isfinite(squared) or squared<=0:raise ValueError('invalid_pepper_sphere_fit')
        radius=float(math.sqrt(squared))
        if not radius_range[0]<=radius<=radius_range[1]:raise ValueError('pepper_radius_outside_observed_envelope')
        error=abs(np.linalg.norm(q-value[:3],axis=1)-radius)
        selected=q[error<.002]
        if len(selected)<minimum:raise ValueError('noisy_pepper_surface')
    residual=float(np.quantile(error,.95))
    if residual>.0025:raise ValueError('pepper_surface_residual')
    return value[:3]+origin,radius,dict(points=len(p),inliers=len(selected),residual_p95_m=residual)


def fit_visible_known_radius(points,radius,expected):
    """Fit a current partial curved patch using this object's measured radius.

    A small amount of wrist/edge clutter is rejected explicitly. Hidden points
    are never synthesized, and the centre must come from current real depth.
    """
    from scipy.optimize import least_squares
    p=np.asarray(points,float);expected=np.asarray(expected,float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<60 or not np.isfinite(p).all():
        raise ValueError('pepper_current_curved_patch_not_observed')
    selected=p;center=expected.copy()
    for _ in range(4):
        fit=least_squares(lambda c:np.linalg.norm(selected-c,axis=1)-radius,
                          center,loss='soft_l1',f_scale=.001,max_nfev=100)
        if not fit.success:raise ValueError('pepper_partial_curved_fit_failed')
        center=fit.x;error=abs(np.linalg.norm(p-center,axis=1)-radius);selected=p[error<.002]
        if len(selected)<60 or len(selected)<.8*len(p):raise ValueError('pepper_partial_patch_clutter_excessive')
    singular=np.linalg.svd(selected-selected.mean(0),compute_uv=False)
    if singular[-1]<.001 or singular[-1]/singular[0]<.025:raise ValueError('pepper_partial_curvature_not_observed')
    residual=float(np.quantile(abs(np.linalg.norm(selected-center,axis=1)-radius),.95))
    return center,radius,dict(points=len(p),inliers=len(selected),residual_p95_m=residual,
        rejected_current_points=len(p)-len(selected),current_partial_surface_measured=True,
        radius_source='this_object_source_rgbd',center_source='current_curved_depth_patch',hidden_points_used=False)


def colour_model(rgb,mask):
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);pixels=hsv[mask]
    if len(pixels)<40:raise ValueError('pepper_source_colour_not_observed')
    hue=int(np.bincount(pixels[:,0],minlength=180).argmax())
    delta=abs(pixels[:,0].astype(float)-hue);delta=np.minimum(delta,180-delta)
    # A low-saturation gray cap does not carry a reliable hue.
    return dict(hue=hue,hue_tolerance=float(min(30,max(8,np.quantile(delta,.95)+4))),
        use_hue=bool(np.median(pixels[:,1])>35),
        saturation_range=[float(max(0,np.quantile(pixels[:,1],.03)-12)),float(min(255,np.quantile(pixels[:,1],.97)+15))],
        value_range=[float(max(15,np.quantile(pixels[:,2],.03)-25)),float(min(255,np.quantile(pixels[:,2],.97)+30))],
        source_pixels=len(pixels))


def colour_mask(rgb,model):
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask=(hsv[:,:,1]>=model['saturation_range'][0])&(hsv[:,:,1]<=model['saturation_range'][1])
    mask&=(hsv[:,:,2]>=model['value_range'][0])&(hsv[:,:,2]<=model['value_range'][1])
    if model['use_hue']:
        delta=abs(hsv[:,:,0].astype(float)-model['hue']);delta=np.minimum(delta,180-delta)
        mask&=delta<=model['hue_tolerance']
    return mask


def source_support_plane(measured,snapshot):
    stamp=snapshot['streams']['depth']['stamp_ns']
    tf=snapshot['tracking_transforms']['map_from_base_at_depth']
    coef=np.asarray(measured['plane_map'],float)
    if (measured['source_depth_stamp_ns']!=stamp or tf['stamp_ns']!=stamp
            or coef.shape!=(3,) or not np.isfinite(coef).all()
            or not 0<=measured['residual_p95_m']<=.004 or measured['plane_points']<150):
        raise ValueError('same_frame_measured_source_table_required')
    normal=np.r_[-coef[:2],1.];local=rotation(tf['quaternion']).T@normal
    if local[2]<=.95:raise ValueError('source_table_not_horizontal')
    return np.r_[-local[:2]/local[2],(coef[2]-normal@tf['translation'])/local[2]]


def source_surface(rgb,depth,snapshot,detection,*,measured_support=None):
    from object_surface_geometry import support_plane
    from upright_cylinder_geometry import select_depth_surface
    if detection.get('name')!=NAME or not .1<=detection.get('confidence',-1)<=1:
        raise ValueError('ground_pepper_detection_proposal_required')
    if rgb.shape!=(*depth.shape,3):raise ValueError('pepper_rgbd_shape_mismatch')
    stamps=[snapshot['streams'][k]['stamp_ns'] for k in ('rgb','depth')]
    if any(type(x) is not int or x<=0 for x in stamps) or abs(stamps[0]-stamps[1])>30_000_000:
        raise ValueError('pepper_unsynchronised_rgbd')
    k=np.asarray(snapshot['rgb_k'],float).reshape(3,3)
    reg=snapshot.get('rgb_from_depth',{})
    if (min(k[0,0],k[1,1])<=0 or not np.isfinite(k).all()
            or np.any(np.abs(snapshot.get('rgb_d',[0]*5))>1e-9)
            or np.asarray(reg.get('translation')).shape!=(3,)
            or not np.isfinite(reg.get('translation')).all()
            or np.linalg.norm(reg['translation'])>1e-6
            or not np.allclose(rotation(reg['quaternion']),np.eye(3),atol=1e-6,rtol=0)
            or not np.allclose(snapshot.get('depth_k',snapshot['rgb_k']),snapshot['rgb_k'],atol=1e-6,rtol=0)):
        raise ValueError('pepper_registered_undistorted_depth_required')
    box=np.asarray(detection['box'],float);h,w=depth.shape
    if box.shape!=(4,) or not np.isfinite(box).all():raise ValueError('pepper_invalid_detection_box')
    x1,y1,x2,y2=box
    if not 1<x1<x2<w-1 or not 1<y1<y2<h-1 or min(x2-x1,y2-y1)<15:
        raise ValueError('pepper_clipped_or_small_detection')
    pad=max(25.,.6*max(x2-x1,y2-y1))
    window,u,v=pixel_window(depth.shape,x1-pad,y1-pad,x2+pad,y2+pad)
    z=depth[window];valid=np.isfinite(z)&(z>0)&(z<3.)
    camera=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),axis=-1)
    tf=snapshot['base_from_rgb'];points=camera@rotation(tf['quaternion']).T+tf['translation']
    inside=(u>=x1+1)&(u<x2-1)&(v>=y1+1)&(v<y2-1)
    ring=valid&~((u>=x1-5)&(u<x2+5)&(v>=y1-5)&(v<y2+5))
    if measured_support is None:plane,_,residual=support_plane(points[ring])
    else:
        plane=source_support_plane(measured_support,snapshot)
        local=points[ring];errors=abs(local[:,:2]@plane[:2]+plane[2]-local[:,2]);inliers=local[errors<=.004]
        if len(inliers)<150 or min(np.ptp(inliers[:,:2],axis=0))<.04:
            raise ValueError('source_table_not_observed_near_target')
        residual=float(np.quantile(abs(inliers[:,:2]@plane[:2]+plane[2]-inliers[:,2]),.95))
    heights=points[:,:,2]-(points[:,:,:2]@plane[:2]+plane[2])
    foreground=valid&inside&(heights>.004)&(heights<HEIGHT_RANGE[1]+.02)
    seeds=(u>=x1+.3*(x2-x1))&(u<x1+.7*(x2-x1))&(v>=y1+.25*(y2-y1))&(v<y1+.75*(y2-y1))
    selected,segmentation=select_depth_surface(points,foreground,seeds)
    return points,heights,selected,plane,residual,segmentation,window


def estimate_source(rgb,depth,snapshot,detection,*,measured_support=None):
    points,z,selected,plane,residual,segmentation,window=source_surface(rgb,depth,snapshot,detection,measured_support=measured_support)
    height=float(np.quantile(z[selected],.99))
    if not HEIGHT_RANGE[0]<=height<=HEIGHT_RANGE[1]:raise ValueError('pepper_height_outside_observation_envelope')
    body_mask=selected&(z>.18*height)&(z<.55*height)
    cap_mask=selected&(z>.73*height)&(z<1.01*height)
    body,r,body_fit=fit_sphere(points[body_mask],BODY_RANGE)
    cap,cr,cap_fit=fit_sphere(points[cap_mask],CAP_RANGE)
    distance=float(np.linalg.norm(cap-body));axis=(cap-body)/distance
    bottom=float(np.r_[body[:2],1.]@plane);body_above=body[2]-bottom
    if (not .015<distance<.06 or cr>r*.85 or axis[2]<math.cos(math.radians(10))
            or not .0175+.004<body_above<r or body_above>height*.6):
        raise ValueError('pepper_wide_body_neck_or_table_geometry_inconsistent')
    appearance=dict(version=1,class_name=NAME,source='actual_verified_ground_pepper_rgbd',
        body=colour_model(rgb[window],body_mask),cap=colour_model(rgb[window],cap_mask),
        new_target_identification_supported=False)
    return dict(class_name=NAME,shape='rounded_body_and_smaller_cap',adapter='round_shaker_rgbd',
        center_m=body.tolist(),cap_center_m=cap.tolist(),body_radius_m=r,cap_radius_m=cr,
        footprint_radius_m=r,height_m=height,body_above_support_m=float(body_above),
        body_cap_distance_m=distance,axis_base_m=axis.tolist(),support_plane_base=plane.tolist(),
        support_height_m=bottom,support_plane_residual_p95_m=residual,
        body_fit=body_fit,cap_fit=cap_fit,appearance=appearance,surface_segmentation=segmentation,
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],confidence=detection['confidence'],
        source='rgbd',frame_id='base_link',depth_valid=True,support_consistent=True,
        estimated_grasp_width_m=2*r,room_confirmed=False,ready_for_motion=False,
        position_source='current_body_and_cap_sphere_depth_fits',hidden_surfaces_measured=False,
        grasp_center_semantics='observed_wide_body_center_not_neck_or_full_height_midpoint',motion_authorized=False)


def validate_reference(ref):
    if ref.get('class_name')!=NAME or ref.get('shape')!='rounded_body_and_smaller_cap':
        raise ValueError('verified_ground_pepper_reference_required')
    if (not BODY_RANGE[0]<=ref['body_radius_m']<=BODY_RANGE[1]
            or not CAP_RANGE[0]<=ref['cap_radius_m']<=CAP_RANGE[1]
            or not HEIGHT_RANGE[0]<=ref['height_m']<=HEIGHT_RANGE[1]
            or not .015<ref['body_cap_distance_m']<.06
            or ref['appearance'].get('source')!='actual_verified_ground_pepper_rgbd'):
        raise ValueError('invalid_ground_pepper_shape_reference')


def surface_extent(body,cap,radius,cap_radius,body_above):
    direction=cap-body;distance=np.linalg.norm(direction);axis=direction/distance
    # The rounded body is truncated by its observed base, rather than treating
    # its fitted sphere's hidden lower segment as the physical object bottom.
    flat_radius=math.sqrt(max(0.,radius**2-body_above**2))
    sine=float(np.linalg.norm(axis[:2]))
    if radius*axis[2]<=body_above:
        lowest=body-np.array([0.,0.,radius])
    else:
        lowest=body-body_above*axis
        if sine>1e-8:lowest-=flat_radius*(np.array([0.,0.,1.])-axis[2]*axis)/sine
    cap_lowest=cap-np.array([0.,0.,cap_radius])
    if cap_lowest[2]<lowest[2]:lowest=cap_lowest
    support_radius=max(radius,float(np.linalg.norm(direction[:2]))+cap_radius)
    return dict(axis_base_m=axis.tolist(),tilt_deg=float(math.degrees(math.acos(np.clip(axis[2],-1,1)))),
        estimated_lowest_point_base_m=lowest.tolist(),estimated_bottom_z_m=float(lowest[2]),
        bottom_vertical_extent_m=float(body[2]-lowest[2]),horizontal_support_radius_m=support_radius)


def measure_near(rgb,depth,snapshot,expected_body,expected_cap,ref):
    validate_reference(ref)
    if rgb.shape!=(*depth.shape,3):raise ValueError('pepper_rgbd_shape_mismatch')
    expected_body=np.asarray(expected_body,float);expected_cap=np.asarray(expected_cap,float)
    if (expected_body.shape!=(3,) or expected_cap.shape!=(3,)
            or not np.isfinite([*expected_body,*expected_cap]).all()):raise ValueError('invalid_pepper_roi_prediction')
    window,u,v=near_window(depth.shape,snapshot,[expected_body,expected_cap],
        [ref['body_radius_m']+.012,ref['cap_radius_m']+.012])
    if u.size==0:raise ValueError('pepper_surface_not_observed')
    z=depth[window];k=np.asarray(snapshot['rgb_k'],float).reshape(3,3);tf=snapshot['base_from_rgb']
    camera=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),axis=-1)
    points=camera@rotation(tf['quaternion']).T+tf['translation']
    local_rgb=rgb[window];valid=np.isfinite(z)&(z>0)&(z<3.)
    results=[]
    direction=expected_cap-expected_body;length=float(np.linalg.norm(direction))
    if not .015<length<.07:raise ValueError('invalid_pepper_body_cap_prediction')
    axis=direction/length
    for name,expected,radius in (('body',expected_body,ref['body_radius_m']),('cap',expected_cap,ref['cap_radius_m'])):
        selected=valid&colour_mask(local_rgb,ref['appearance'][name])
        selected&=np.linalg.norm(points-expected,axis=-1)<radius+.012
        coloured_current=selected.copy()
        axial=(points-expected_body)@axis
        if name=='body':
            selected&=(axial>.18*ref['height_m']-ref['body_above_support_m'])
            upper=.55*ref['height_m']-ref['body_above_support_m']
            original=selected&(axial<upper)
            # A wrist may hide all but the curved shoulder. Collect additional
            # REAL surface points in a further2mm band, staying below the neck.
            # Sphere radius, curvature, residual and minimum60 points stay intact.
            expanded=np.count_nonzero(original)<60
            selected=selected&(axial<min(upper+.002,radius*.55)) if expanded else original
        else:
            selected&=(points-expected_cap)@axis>-.003
        try:center,actual_radius,fit=fit_sphere(points[selected],[radius-.003,radius+.003])
        except ValueError as exc:
            if name!='body' or str(exc) not in ('pepper_surface_residual','pepper_radius_outside_observed_envelope'):
                raise
            # A table-supported shaker may settle upright after a glancing
            # contact. Do not keep cutting its new depth with the OLD tilted
            # axis. Reacquire a gravity-aligned belly belt from real points.
            vertical=points[:,:,2]-expected_body[2]
            belt=coloured_current&(vertical>.18*ref['height_m']-ref['body_above_support_m'])
            belt&=vertical<.55*ref['height_m']-ref['body_above_support_m']
            try:center,actual_radius,fit=fit_sphere(points[belt],[radius-.003,radius+.003])
            except ValueError:center,actual_radius,fit=fit_visible_known_radius(points[belt],radius,expected)
            fit['current_gravity_belt_reacquired']=True
        if name=='body' and expanded:fit['curved_shoulder_band_extended_m']=.002
        if np.linalg.norm(center-expected)>.035:raise ValueError('current_pepper_surface_outside_association')
        results.append((center,actual_radius,fit))
    (body,r,bfit),(cap,cr,cfit)=results;distance=float(np.linalg.norm(cap-body))
    if abs(distance-ref['body_cap_distance_m'])>.006:raise ValueError('pepper_body_cap_pair_inconsistent')
    return dict(class_name=NAME,shape='rounded_body_and_smaller_cap',center_m=body.tolist(),cap_center_m=cap.tolist(),
        body_radius_m=r,cap_radius_m=cr,footprint_radius_m=r,height_m=ref['height_m'],
        body_above_support_m=ref['body_above_support_m'],body_cap_distance_m=distance,
        body_fit=bfit,cap_fit=cfit,**surface_extent(body,cap,r,cr,ref['body_above_support_m']),
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],hidden_surfaces_measured=False,
        position_source='current_body_and_cap_sphere_depth_fits',predicted_center_returned=False,motion_authorized=False)


def near_window(shape,snapshot,centers,radii):
    """Project enclosing 3D cubes; retain every pixel in the original spheres."""
    k=np.asarray(snapshot['rgb_k'],float).reshape(3,3);tf=snapshot['base_from_rgb']
    if not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0:raise ValueError('invalid_appearance_intrinsics')
    optical=(np.asarray(centers)-tf['translation'])@rotation(tf['quaternion'])
    signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    vertices=(optical[:,None,:]+np.asarray(radii)[:,None,None]*signs).reshape(-1,3)
    if np.any(vertices[:,2]<=.05):return pixel_window(shape,0,0,shape[1],shape[0])
    pixels=vertices@k.T;pixels=pixels[:,:2]/pixels[:,2,None]
    lo=np.floor(pixels.min(0))-2;hi=np.ceil(pixels.max(0))+3
    return pixel_window(shape,*lo,*hi)


def lift_follow(before,bs,after,ass,opening):
    from canned_grasp_geometry import transform_point
    if bs.get('tracking_epoch')!=ass.get('tracking_epoch') or after['stamp_ns']<=before['stamp_ns']:
        raise ValueError('pepper_lift_epoch_or_time_mismatch')
    p0=transform_point(bs,bs['palm_base_at_depth']);p1=transform_point(ass,ass['palm_base_at_depth'])
    body_delta=transform_point(ass,after['center_m'])-transform_point(bs,before['center_m'])
    cap_delta=transform_point(ass,after['cap_center_m'])-transform_point(bs,before['cap_center_m'])
    palm_delta=p1-p0;joint_delta=ass['joints']['arm_lift_joint']-bs['joints']['arm_lift_joint']
    current=ass['joints']['hand_motor_joint']
    if not .02<opening<=.62 or abs(current-opening)>.03:raise ValueError('pepper_grip_not_preserved')
    error=max(float(np.linalg.norm(body_delta-palm_delta)),float(np.linalg.norm(cap_delta-palm_delta)))
    if min(body_delta[2],cap_delta[2],palm_delta[2],joint_delta)<.03 or error>.02:
        raise ValueError('pepper_did_not_follow_palm_off_table')
    return dict(stamp_ns=after['stamp_ns'],object_palm_follow_error_m=error,
        object_delta_odom_m=body_delta.tolist(),cap_delta_odom_m=cap_delta.tolist(),palm_delta_odom_m=palm_delta.tolist(),
        grasp_lift_verified=True,placement_verified=False)

"""RGB continuation evidence for an ALREADY RGBD-verified grasped can.

Not a detector for new objects. Projection selects a ROI, never measures3D.
Actual source RGBD isolates body appearance; actual current pixels must contain
both that body appearance and a compatible metallic cap near the current hand.
"""
import math
import cv2
import numpy as np

from canned_carry_geometry import tucked_holding
from hand_camera_ray import rotation


def pixel_points(depth,snapshot):
    k=np.asarray(snapshot['rgb_k'],float).reshape(3,3)
    if not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0:raise ValueError('invalid_appearance_intrinsics')
    v,u=np.indices(depth.shape)
    camera=np.stack(((u-k[0,2])*depth/k[0,0],(v-k[1,2])*depth/k[1,1],depth),axis=-1)
    tf=snapshot['base_from_rgb']
    return camera@rotation(tf['quaternion']).T+np.asarray(tf['translation'],float)


def build_appearance(rgb,depth,snapshot,measurement):
    if measurement.get('class_name')!='canned_juice':raise ValueError('verified_can_template_required')
    if rgb.shape[:2]!=depth.shape:raise ValueError('template_rgbd_shape_mismatch')
    points=pixel_points(depth,snapshot);hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    c=np.asarray(measurement['center_m'],float);radius=measurement['footprint_radius_m']
    height=measurement['height_m']
    axis=np.asarray(measurement.get('axis_base_m',[0.,0.,1.]),float);axis/=np.linalg.norm(axis)
    delta=points-c;axial=delta@axis;radial=np.linalg.norm(delta-axial[:,:,None]*axis,axis=2)
    inside=(depth>0)&np.isfinite(depth)&(radial<radius+.004)
    body=inside&(axial>-.4*height)&(axial<height/2-.005)&(hsv[:,:,1]>70)&(hsv[:,:,2]>30)
    if body.sum()<100:raise ValueError('insufficient_verified_can_body_appearance')
    values=hsv[:,:,0][body];hue=int(np.bincount(values,minlength=180).argmax())
    distance=np.abs(values.astype(float)-hue);distance=np.minimum(distance,180-distance)
    tolerance=float(max(6.,np.quantile(distance,.95)+3))
    if tolerance>18:raise ValueError('ambiguous_source_can_body_color')
    cap=inside&(np.abs(axial-height/2)<.006)&(hsv[:,:,1]<80)
    if cap.sum()<30:raise ValueError('insufficient_verified_metal_cap_appearance')
    return dict(version=1,class_name='canned_juice',source='actual_verified_can_rgbd',
        hue=hue,hue_tolerance=tolerance,saturation_min=70,value_min=30,
        silver_saturation_max=80,silver_value_min=float(max(70,np.quantile(hsv[:,:,2][cap],.1)-20)),
        source_body_pixels=int(body.sum()),source_cap_pixels=int(cap.sum()),
        new_target_identification_supported=False,motion_authorized=False)


def project_held_can(snapshot,reference):
    if reference.get('can_grasp_and_lift_verified') is not True:raise ValueError('verified_grasp_required_for_rgb_continuation')
    palm=np.asarray(snapshot['palm_base_at_depth'],float)
    center=palm+np.asarray(reference['held_center_minus_palm_base_m'],float)
    r=reference['can_footprint_radius_m'];h=reference['can_height_m']
    if not np.isfinite([*center,r,h]).all() or not .01<=r<=.06 or not .035<=h<=.20:
        raise ValueError('invalid_held_can_dimensions')
    tf=snapshot['base_from_rgb'];rot=rotation(tf['quaternion']);origin=np.asarray(tf['translation'],float)
    k=np.asarray(snapshot['rgb_k'],float).reshape(3,3)
    def project(points):
        optical=(np.asarray(points)-origin)@rot
        if np.any(optical[:,2]<=.05):raise ValueError('held_can_behind_camera')
        pixels=optical@k.T;return pixels[:,:2]/pixels[:,2,None],optical[:,2]
    theta=np.linspace(0,2*math.pi,64,endpoint=False)
    from tracked_can_cap_geometry import basis
    axis,u,v=basis(reference.get('can_axis_base_m',[0.,0.,1.]))
    cap=center+np.cos(theta)[:,None]*r*u+np.sin(theta)[:,None]*r*v+axis*h/2
    cap_pixels,cap_depth=project(cap)
    center_pixel,center_depth=project([center+axis*h/2])
    ellipse=cv2.fitEllipse(cap_pixels.astype(np.float32))
    corners=center+np.array([x*r*u+y*r*v+z*h/2*axis for x in (-1,1) for y in (-1,1) for z in (-1,1)])
    box_pixels,box_depth=project(corners)
    width=snapshot['streams']['rgb']['width'];height=snapshot['streams']['rgb']['height']
    lo=np.floor(box_pixels.min(0)-18).astype(int);hi=np.ceil(box_pixels.max(0)+18).astype(int)
    lo=np.maximum(lo,[0,0]);hi=np.minimum(hi,[width,height])
    if np.any(hi-lo<20):raise ValueError('held_can_not_in_camera_view')
    return dict(roi=[*lo.tolist(),*hi.tolist()],cap_center_pixel=center_pixel[0].tolist(),
        cap_ellipse_center=list(map(float,ellipse[0])),cap_axes=list(map(float,ellipse[1])),
        cap_camera_depth_prediction_m=float(center_depth[0]),
        expected_depth_range_prediction_m=[float(box_depth.min()),float(box_depth.max())],
        semantics='palm_and_pregrasp_dimensions_projection_only_not_measured_object_position')


def assess(rgb,depth,snapshot,reference,appearance):
    if appearance.get('source')!='actual_verified_can_rgbd' or appearance.get('class_name')!='canned_juice':
        raise ValueError('verified_can_appearance_model_required')
    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    joints=[float(snapshot['joints'][key]) for key in names];holding=reference['held_opening_rad']
    if not tucked_holding(joints,holding) or not .02<holding<=.62:
        raise ValueError('tucked_actual_grip_required_for_rgb_continuation')
    if rgb.shape[:2]!=depth.shape:raise ValueError('held_rgb_depth_shape_mismatch')
    valid=np.isfinite(depth)&(depth>0)
    if np.count_nonzero(valid)<1000:raise ValueError('depth_sensor_not_observing_scene')
    projection=project_held_can(snapshot,reference)
    if projection['cap_camera_depth_prediction_m']>=.415:raise ValueError('rgb_continuation_only_for_near_clipped_held_can')
    u,v=np.rint(projection['cap_center_pixel']).astype(int)
    if not 0<=u<depth.shape[1] or not 0<=v<depth.shape[0]:raise ValueError('projected_held_cap_outside_image')
    tile=depth[max(0,v-10):v+11,max(0,u-10):u+11]
    zero_fraction=float(np.mean(~np.isfinite(tile)|(tile<=0)))
    if zero_fraction<.5:raise ValueError('use_actual_rgbd_when_cap_depth_available')
    x1,y1,x2,y2=projection['roi'];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    hue_delta=np.abs(hsv[:,:,0].astype(float)-appearance['hue']);hue_delta=np.minimum(hue_delta,180-hue_delta)
    roi=np.zeros(depth.shape,np.uint8);roi[y1:y2,x1:x2]=1
    body=((hue_delta<=appearance['hue_tolerance'])&(hsv[:,:,1]>=appearance['saturation_min'])
        &(hsv[:,:,2]>=appearance['value_min'])&(roi>0)).astype(np.uint8)
    body=cv2.morphologyEx(body,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    _,labels,stats,centers=cv2.connectedComponentsWithStats(body,connectivity=8)
    axes=np.sort(projection['cap_axes']);expected_area=math.pi*axes.prod()/4
    candidates=[(i,st,centers[i]) for i,st in enumerate(stats) if i>0 and st[4]>=max(100,.08*expected_area)
        and st[2]>=.4*axes[1] and st[3]>=.2*axes[0]]
    if not candidates:raise ValueError('unique_current_can_body_appearance_not_seen')
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);edges=cv2.Canny(gray,40,120);edges[roi==0]=0
    contours,_=cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    expected_center=np.asarray(projection['cap_ellipse_center']);accepted=[]
    silver=(hsv[:,:,1]<=appearance['silver_saturation_max'])&(hsv[:,:,2]>=appearance['silver_value_min'])
    for contour in contours:
        if len(contour)<20:continue
        ellipse=cv2.fitEllipse(contour);center=np.asarray(ellipse[0]);measured_axes=np.sort(ellipse[1])
        if not np.isfinite([*center,*measured_axes]).all() or min(measured_axes)<15:continue
        ratios=measured_axes/axes;center_error=float(np.linalg.norm(center-expected_center)/axes[1])
        if np.any(ratios<.55) or np.any(ratios>1.6) or center_error>.35:continue
        disk=np.zeros(depth.shape,np.uint8);cv2.ellipse(disk,ellipse,1,-1)
        silver_fraction=float(np.mean(silver[disk>0]))
        if silver_fraction<.2:continue
        score=center_error+float(np.linalg.norm(np.log(ratios)))
        for label,body_box,body_center in candidates:
            if (abs(body_center[0]-center[0])>.55*measured_axes[1]
                    or body_center[1]-center[1]<.1*measured_axes[0]
                    or body_box[1]>center[1]+.8*measured_axes[0]
                    or not .45<=body_box[2]/measured_axes[1]<=1.6):continue
            accepted.append(dict(score=score,center_pixel=center.tolist(),axes_pixel=measured_axes.tolist(),
                center_error_normalized=center_error,silver_fraction=silver_fraction,
                body_label=label,body_box=body_box,body_center=body_center))
    if not accepted:raise ValueError('current_metal_cap_not_seen_with_can_body')
    if len({e['body_label'] for e in accepted})!=1:raise ValueError('ambiguous_body_cap_pairs_in_hand_roi')
    cap=min(accepted,key=lambda e:e['score'])
    label=cap.pop('body_label');body_box=cap.pop('body_box');body_center=cap.pop('body_center')
    component_depth=depth[labels==label];available=component_depth[np.isfinite(component_depth)&(component_depth>0)]
    if len(available)>=30 and np.median(available)>projection['expected_depth_range_prediction_m'][1]+.08:
        raise ValueError('current_body_depth_contradicts_held_location')
    return dict(status='already_held_can_rgb_appearance_observed',class_name='canned_juice',
        evidence_type='current_body_color_and_cap_shape_bound_to_verified_grasp',
        body_box_pixel=body_box[:4].tolist(),body_center_pixel=body_center.tolist(),body_pixels=int(body_box[4]),
        cap=cap,projection_roi=projection,near_cap_zero_depth_fraction=zero_fraction,
        source_identity='prior_rgbd_verified_grasp_not_new_detector_identity',
        current_rgb_appearance_verified=True,current_depth_surface_verified=False,
        measured_3d_object_position_available=False,new_target_identity_created=False,
        measured_center_base_m=None,detector_reran=False,room_confirmed=False,
        placement_verified=False,motion_authorized=False,
        limitation='RGB held-object continuation only. Prediction selects ROI; no depth0 coordinates, no new object search or full3D hold reconstruction.')

"""Hand-RGB pose of this already identified can using its measured dimensions.

Metric pose is a known-size silhouette fit, NOT a depth-camera measurement.
Uses the existing checked SIGVerse hand raster rotation. No robot commands.
"""
import math
import cv2
import numpy as np
from scipy.optimize import least_squares
from hand_camera_ray import rotation,HAND_RASTER_TO_FRAME


def hand_feedback_fresh(snapshot,state,now_ns,epoch):
    """This hand-only controller does not depend on head image age."""
    from task_outcome_policy import pose_matches
    if snapshot.get('tracking_epoch')!=epoch:raise ValueError('hand_trial_epoch_mismatch')
    stamp=snapshot['streams']['hand']['stamp_ns']
    if type(stamp) is not int or not 0<=now_ns-stamp<=400_000_000:
        raise ValueError('fresh_hand_image_required')
    if not -20_000_000<=now_ns-state['stamp_ns']<=600_000_000:
        raise ValueError('fresh_hand_trial_joint_feedback_required')
    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    if not pose_matches(state['joints'],[float(snapshot['joints'][k]) for k in names]):
        raise ValueError('arm_changed_since_hand_image')
    return dict(hand_age_s=(now_ns-stamp)/1e9,joint_age_s=(now_ns-state['stamp_ns'])/1e9,
                head_image_not_used=True,original_image_stamp_ns=stamp)


def project(snapshot,points):
    tf=snapshot['base_from_hand'];r=rotation(tf['quaternion'])@HAND_RASTER_TO_FRAME
    optical=(np.asarray(points)-tf['translation'])@r
    if np.any(optical[:,2]<=.005):raise ValueError('can_model_crosses_hand_camera')
    k=np.asarray(snapshot['hand_k'],float).reshape(3,3)
    pixels=optical@k.T
    return pixels[:,:2]/pixels[:,2,None]


def cylinder_points(center,axis,radius,length):
    axis=np.asarray(axis,float);axis/=np.linalg.norm(axis)
    across=np.array([-axis[1],axis[0],0.]);across/=np.linalg.norm(across)
    angles=np.linspace(0,2*math.pi,96,endpoint=False)
    ring=radius*np.cos(angles)[:,None]*across+radius*np.sin(angles)[:,None]*[0.,0.,1.]
    return np.concatenate([np.asarray(center)+ring+axial*axis for axial in (-length/2,length/2)])


def mask_body(hand,appearance,*,allow_clipped=False):
    hsv=cv2.cvtColor(hand,cv2.COLOR_BGR2HSV)
    dh=abs(hsv[:,:,0].astype(float)-appearance['hue']);dh=np.minimum(dh,180-dh)
    mask=((dh<=appearance['tolerance'])&(hsv[:,:,1]>70)&(hsv[:,:,2]>30)).astype(np.uint8)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    candidates=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=100]
    if len(candidates)!=1:raise ValueError('unique_same_can_hand_colour_component_required')
    selected=labels==candidates[0];v,u=np.nonzero(selected)
    h,w=hand.shape[:2]
    if not allow_clipped and (min(u)<2 or max(u)>w-3 or min(v)<2 or max(v)>h-3):
        raise ValueError('hand_can_silhouette_clipped')
    return selected,np.c_[u,v].astype(float)


def fit(hand,snapshot,model,appearance,*,height=None,axis_hint=None,center_hint=None):
    if snapshot['streams']['hand']['frame']!='hand_camera_rgb_frame' or np.any(np.asarray(snapshot['hand_d'])!=0):
        raise ValueError('reviewed_sigverse_hand_raster_required')
    mask,observed=mask_body(hand,appearance)
    tf=snapshot['base_from_hand'];r=rotation(tf['quaternion'])@HAND_RASTER_TO_FRAME
    k=np.asarray(snapshot['hand_k'],float).reshape(3,3)
    c=np.median(observed,axis=0)
    ray=r@np.linalg.solve(k,[*c,1.])
    camera=np.asarray(tf['translation']);radius=float(model['radius_m'])
    z=float(model['center_base_m'][2] if height is None else height)
    if abs(ray[2])<.1:raise ValueError('hand_view_not_downward_enough')
    initial=camera+ray*((z+radius*.5-camera[2])/ray[2])
    initial[2]=z
    if center_hint is not None:
        hint=np.asarray(center_hint,float)
        if np.linalg.norm(initial[:2]-hint[:2])>.15:raise ValueError('hand_component_outside_same_can_search_region')
    axis=np.asarray(model['axis_base'] if axis_hint is None else axis_hint,float)
    yaw=math.atan2(axis[1],axis[0])
    length=float(model.get('coloured_length_m',model['observed_length_m']*.92))
    directions=np.c_[np.cos(np.linspace(0,2*math.pi,32,endpoint=False)),np.sin(np.linspace(0,2*math.pi,32,endpoint=False))]
    obs_support=(observed@directions.T).max(0)
    free=height is None
    def unpack(q):
        center=np.array([q[0],q[1],q[2] if free else height])
        angle=q[3] if free else q[2];size=q[4] if free else q[3]
        return center,np.array([math.cos(angle),math.sin(angle),0.]),size
    def residual(q):
        center,ax,size=unpack(q)
        pixels=project(snapshot,cylinder_points(center,ax,radius,size))
        return (pixels@directions.T).max(0)-obs_support
    x0=[*initial[:2],z,yaw,length] if free else [*initial[:2],yaw,length]
    zlo=float(model['center_base_m'][2]-.025)
    zhi=float(camera[2]-radius-.018)
    if free and not zlo<z<zhi:raise ValueError('hand_fit_initial_height_outside_view')
    low=[initial[0]-.12,initial[1]-.12]+([zlo] if free else [])+[yaw-.30,length*.94]
    high=[initial[0]+.12,initial[1]+.12]+([zhi] if free else [])+[yaw+.30,length*1.06]
    result=least_squares(residual,x0,bounds=(low,high),loss='soft_l1',f_scale=2.,max_nfev=150)
    errors=abs(residual(result.x));center,ax,size=unpack(result.x)
    error=float(np.quantile(errors,.95))
    if error>5.:raise ValueError('hand_can_silhouette_model_disagrees:'+str(round(error,3)))
    return dict(class_name='canned_juice',center_m=center.tolist(),axis_base=ax.tolist(),
        source='current_hand_rgb_known_size_cylinder_fit',depth_measured=False,
        dimensions_source='this_trial_initial_head_rgbd',height_constraint='none' if free else 'previously_measured_table_supported_can_height',
        silhouette_residual_p95_pixels=error,observed_pixels=len(observed),
        coloured_length_m=float(size),radius_m=radius,
        stamp_ns=snapshot['streams']['hand']['stamp_ns'],motion_authorized=False,
        hidden_surfaces_measured=False)


def follows_lift(before,after,appearance):
    """Actual can texture stays with wrist; an unlifted can shrinks in view."""
    old_mask,_=mask_body(before,appearance,allow_clipped=True)
    new_mask,_=mask_body(after,appearance,allow_clipped=True)
    sift=cv2.SIFT_create(nfeatures=900)
    pa,da=sift.detectAndCompute(cv2.cvtColor(before,cv2.COLOR_BGR2GRAY),old_mask.astype(np.uint8)*255)
    pb,db=sift.detectAndCompute(cv2.cvtColor(after,cv2.COLOR_BGR2GRAY),new_mask.astype(np.uint8)*255)
    if da is None or db is None:raise ValueError('can_texture_for_lift_not_observed')
    matches=[first for first,second in cv2.BFMatcher().knnMatch(da,db,k=2) if first.distance<.70*second.distance]
    if len(matches)<8:raise ValueError('insufficient_same_can_texture_matches')
    a=np.asarray([pa[m.queryIdx].pt for m in matches],np.float32)
    b=np.asarray([pb[m.trainIdx].pt for m in matches],np.float32)
    affine,inliers=cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=3.)
    if affine is None or inliers.sum()<8:raise ValueError('same_can_texture_motion_not_consistent')
    scale=float(np.linalg.norm(affine[:,0]));movement=float(np.linalg.norm(affine@np.r_[a.mean(0),1.]-a.mean(0)))
    area_ratio=float(new_mask.sum()/old_mask.sum())
    if not .90<=scale<=1.10 or movement>20 or not .70<=area_ratio<=1.4:
        raise ValueError('can_does_not_follow_wrist_during_lift')
    return dict(evidence_type='same_can_texture_relative_to_hand_camera_after_measured_arm_lift',
        matches=len(matches),inliers=int(inliers.sum()),scale=scale,movement_pixels=movement,
        colour_area_ratio=area_ratio,object_depth_measured=False)

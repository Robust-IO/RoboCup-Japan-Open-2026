"""Rigid held-can estimates anchored in this trial's confirmed grasp.

Current hand silhouette associates the already-held object. Geometry returned
here is explicitly kinematic, never a new object-depth observation.
"""
import cv2
import numpy as np
from hand_camera_ray import rotation
from canned_grasp_geometry import transform_point
from horizontal_can_hand import mask_body


def palm_pose(snapshot):
    hand=snapshot['base_from_hand'];local=snapshot['palm_from_hand']
    r=rotation(hand['quaternion'])@rotation(local['quaternion']).T
    p=np.asarray(hand['translation'])-r@np.asarray(local['translation'])
    return p,r


def context_from_contact(snapshot,locked,model,holding):
    center=transform_point(snapshot,locked['center_odom_m'],inverse=True)
    odom=rotation(snapshot['tracking_transforms']['odom_from_base_at_depth']['quaternion'])
    axis=odom.T@np.asarray(locked['axis_odom']);axis/=np.linalg.norm(axis)
    p,r=palm_pose(snapshot)
    return dict(class_name='canned_juice',center_palm_m=(r.T@(center-p)).tolist(),
        axis_palm=(r.T@axis).tolist(),radius_m=model['radius_m'],length_m=model['observed_length_m'],
        holding_opening_rad=holding,position_source='locked_preclose_hand_known_size_pose_and_actual_palm_tf',
        original_hand_observation_stamp_ns=locked['source_hand_stamp_ns'],
        original_dimensions_depth_stamp_ns=model['stamp_ns'],new_object_depth_measured=False)


def cylinder_extents(axis,radius,length):
    axis=np.asarray(axis,float);axis/=np.linalg.norm(axis)
    vertical=float(abs(axis[2])*length/2+np.sqrt(max(0.,1-axis[2]**2))*radius)
    horizontal=float(np.linalg.norm(axis[:2])*length/2+radius)
    return vertical,horizontal


def held_shape(snapshot,held):
    p,r=palm_pose(snapshot);center=p+r@held['center_palm_m'];axis=r@held['axis_palm']
    vertical,horizontal=cylinder_extents(axis,held['radius_m'],held['length_m'])
    return dict(class_name='canned_juice',shape='upright_cylinder' if abs(axis[2])>.95 else 'tilted_cylinder',
        center_m=center.tolist(),height_m=held['length_m'],footprint_radius_m=held['radius_m'],
        axis_base_m=axis.tolist(),horizontal_support_radius_m=horizontal,
        bottom_vertical_extent_m=vertical,estimated_lowest_point_base_m=(center-[0,0,vertical]).tolist(),
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        source='confirmed_rigid_grasp_kinematics_with_current_hand_silhouette',
        object_depth_measured=False,position_is_kinematic_estimate=True)


def held_silhouette(reference,current,appearance):
    old,old_points=mask_body(reference,appearance,allow_clipped=True)
    hsv=cv2.cvtColor(current,cv2.COLOR_BGR2HSV)
    d=abs(hsv[:,:,0].astype(float)-appearance['hue']);d=np.minimum(d,180-d)
    candidate=((d<=appearance['tolerance'])&(hsv[:,:,1]>70)&(hsv[:,:,2]>30)).astype(np.uint8)
    candidate=cv2.morphologyEx(candidate,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    count,labels,stats,centers=cv2.connectedComponentsWithStats(candidate,8)
    associated=[i for i in range(1,count) if np.count_nonzero((labels==i)&old)>.5*old.sum()]
    if len(associated)!=1:raise ValueError('same_held_can_not_seen_in_hand_region')
    new=labels==associated[0];ratio=float(new.sum()/old.sum())
    overlap=float(np.count_nonzero(old&new)/np.count_nonzero(old|new))
    shift=float(np.linalg.norm(centers[associated[0]]-old_points.mean(0)))
    if not .75<=ratio<=1.30 or overlap<.70 or shift>20:
        raise ValueError('held_can_changed_relative_to_hand')
    return dict(evidence_type='current_same_can_silhouette_at_confirmed_grip',
        iou=overlap,area_ratio=ratio,center_shift_pixels=shift,
        source_pixels=int(old.sum()),current_pixels=int(new.sum()),object_depth_measured=False)

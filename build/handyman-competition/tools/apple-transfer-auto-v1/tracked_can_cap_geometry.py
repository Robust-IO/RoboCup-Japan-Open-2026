"""Measured cap-axis tracking of a known grasped can, including modest tilt.

Actual metal-cap depth defines the axis; current source-colored side depth must
independently fit the physical radius. Predictions select a local ROI only.
"""
import math
import cv2
import numpy as np

from upright_cylinder_geometry import fit_side_circle


def basis(axis):
    axis=np.asarray(axis,float);axis=axis/np.linalg.norm(axis)
    u=np.array([1.,0.,0.]);u-=axis*u.dot(axis);u/=np.linalg.norm(u)
    return axis,u,np.cross(axis,u)


def measure(depth,rgb,snapshot,expected,radius,height,appearance):
    from canned_grasp_geometry import depth_points
    if appearance.get('source')!='actual_verified_can_rgbd' or appearance.get('class_name')!='canned_juice':
        raise ValueError('verified_can_appearance_required_for_cap_axis_tracking')
    expected=np.asarray(expected,float)
    if (expected.shape!=(3,) or not np.isfinite([*expected,radius,height]).all()
            or not .01<=radius<=.06 or not .035<=height<=.20 or rgb.shape[:2]!=depth.shape):
        raise ValueError('invalid_cap_axis_can_input')
    points=depth_points(depth,snapshot);valid=np.isfinite(depth)&(depth>0)&(depth<3.)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)[valid]
    radial=np.linalg.norm(points[:,:2]-expected[:2],axis=1)
    metallic=(hsv[:,1]<=appearance['silver_saturation_max'])&(hsv[:,2]>=appearance['silver_value_min'])
    region=(radial<radius+.016)&(abs(points[:,2]-(expected[2]+height/2))<.035)&metallic
    cap=points[region]
    if len(cap)<80:raise ValueError('current_metal_cap_depth_not_observed')
    rng=np.random.default_rng(93);best=None
    for _ in range(160):
        sample=cap[rng.choice(len(cap),3,replace=False)]
        n=np.cross(sample[1]-sample[0],sample[2]-sample[0]);length=np.linalg.norm(n)
        if length<1e-8:continue
        n/=length
        if n[2]<0:n=-n
        if n[2]<math.cos(math.radians(30)):continue
        mask=abs((cap-sample[0])@n)<.0015
        if best is None or mask.sum()>best.sum():best=mask
    if best is None or best.sum()<80:raise ValueError('stable_current_cap_plane_not_observed')
    selected=cap[best]
    for _ in range(2):
        origin=selected.mean(0);_,_,vt=np.linalg.svd(selected-origin,full_matrices=False);axis=vt[-1]
        if axis[2]<0:axis=-axis
        selected=cap[abs((cap-origin)@axis)<.0015]
    if len(selected)<80 or axis[2]<math.cos(math.radians(30)):raise ValueError('can_tilt_outside_current_tracking_model')
    axis,u,v=basis(axis);origin=selected.mean(0)
    projection=np.c_[(selected-origin)@u,(selected-origin)@v]
    if min(np.ptp(projection,axis=0))<min(.02,radius):raise ValueError('current_cap_plane_extent_not_observed')
    delta=np.abs(hsv[:,0].astype(float)-appearance['hue']);delta=np.minimum(delta,180-delta)
    colored=(delta<=appearance['hue_tolerance'])&(hsv[:,1]>=appearance['saturation_min'])&(hsv[:,2]>=appearance['value_min'])
    axial=(points-origin)@axis
    side=points[(axial>-.30*height)&(axial<-.05*height)&(radial<radius+.025)&colored]
    xy=np.c_[(side-origin)@u,(side-origin)@v]
    center2,actual_radius,residual,arc=fit_side_circle(xy,[max(.01,radius-.008),min(.06,radius+.008)])
    nearby_plane_points_excluded=0
    if np.quantile(np.linalg.norm(projection-center2,axis=1),.95)>actual_radius+.008:
        # At deeper seating, a wrist surface can share the metal cap's plane.
        # Restrict it using the independently observed COLORED side circle,
        # then refit actual cap depth. Never accept a predicted or missing cap.
        keep=np.linalg.norm(projection-center2,axis=1)<=actual_radius+.003
        nearby_plane_points_excluded=int((~keep).sum());selected=selected[keep]
        if len(selected)<80:raise ValueError('current_cap_inside_observed_side_not_visible')
        origin=selected.mean(0);_,_,vt=np.linalg.svd(selected-origin,full_matrices=False);axis=vt[-1]
        if axis[2]<0:axis=-axis
        if axis[2]<math.cos(math.radians(30)):raise ValueError('can_tilt_outside_current_tracking_model')
        axis,u,v=basis(axis)
        projection=np.c_[(selected-origin)@u,(selected-origin)@v]
        if (min(np.ptp(projection,axis=0))<min(.02,radius)
                or np.quantile(abs((selected-origin)@axis),.95)>.0015):
            raise ValueError('current_cap_inside_observed_side_not_established')
        axial=(points-origin)@axis
        side=points[(axial>-.30*height)&(axial<-.05*height)&(radial<radius+.025)&colored]
        xy=np.c_[(side-origin)@u,(side-origin)@v]
        center2,actual_radius,residual,arc=fit_side_circle(xy,[max(.01,radius-.008),min(.06,radius+.008)])
    capcenter=origin+center2[0]*u+center2[1]*v;center=capcenter-axis*height/2
    if (np.linalg.norm(center-expected)>.035
            or np.quantile(np.linalg.norm(projection-center2,axis=1),.95)>actual_radius+.008):
        raise ValueError('current_cap_side_or_position_disagree')
    sine=float(np.linalg.norm(axis[:2]));extent=float(height/2*axis[2]+actual_radius*sine)
    low=center-axis*height/2
    if sine>1e-6:low-=actual_radius*(np.array([0.,0.,1.])-axis[2]*axis)/sine
    return dict(class_name='canned_juice',shape='upright_cylinder',model_pose='measured_near_upright_axis',
        center_m=center.tolist(),footprint_radius_m=actual_radius,surface_radius_m=actual_radius,height_m=height,
        axis_base_m=axis.tolist(),tilt_deg=float(math.degrees(math.acos(np.clip(axis[2],-1,1)))),
        cap_center_base_m=capcenter.tolist(),observed_top_z_m=float(capcenter[2]+actual_radius*sine),
        estimated_bottom_z_m=float(low[2]),estimated_lowest_point_base_m=low.tolist(),
        bottom_vertical_extent_m=extent,horizontal_support_radius_m=float(actual_radius+height/2*sine),
        side_points=len(side),top_points=len(selected),cylinder_side_residual_p95_m=residual,observed_side_arc_rad=arc,
        nearby_plane_points_excluded=nearby_plane_points_excluded,
        cap_plane_residual_p95_m=float(np.quantile(abs((selected-origin)@axis),.95)),
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],dimension_source='precontact_rgbd',
        hidden_bottom_measured=False,position_source='current_cap_plane_and_source_colored_side_depth',
        radius_range_relaxed=False,predicted_center_returned=False,detector_reran=False,identity_confirmed=False,motion_authorized=False)

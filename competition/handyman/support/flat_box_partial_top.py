"""Known-size box pose from currently visible top edges after near-field lock.

Occluded edges are not claimed measured. At least one actual outer edge on each
axis must agree with the last full RGBD box within 8mm. No hidden depth is used.
"""
import cv2,numpy as np
from held_can_appearance import pixel_points

def fit_points(points,expected,axis,model):
    p=np.asarray(points,float);c=np.asarray(expected,float);a=np.asarray(axis,float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<40 or not np.isfinite(p).all():raise ValueError('visible_box_top_depth_required')
    a=a.copy();a[2]=0.;a/=np.linalg.norm(a);n=np.array([-a[1],a[0],0.])
    top=float(np.quantile(p[:,2],.98));p=p[abs(p[:,2]-top)<.0025]
    if len(p)<30:raise ValueError('visible_box_top_plane_required')
    center=c.copy();edges=[]
    for direction,size in ((a,model['length_m']),(n,model['width_m'])):
        lo,hi=np.quantile(p@direction,[.01,.99]);middle=float(c@direction)
        if hi-lo<min(.012,.18*size):raise ValueError('visible_box_edge_span_insufficient')
        if lo<middle-size/2-.010 or hi>middle+size/2+.010:raise ValueError('partial_box_outside_locked_footprint')
        candidates=[(float(lo-(middle-size/2)),'minimum'),(float(hi-(middle+size/2)),'maximum')]
        matched=[x for x in candidates if abs(x[0])<=.008]
        if not matched:raise ValueError('actual_outer_box_edge_not_seen')
        shift=float(np.mean([x[0] for x in matched]));center+=direction*shift
        edges.append(dict(visible_edge=[x[1] for x in matched],measured_span_m=float(hi-lo),center_shift_m=shift))
    center[2]=top-model['height_m']/2
    if np.linalg.norm(center-c)>.025:raise ValueError('partial_box_disagrees_with_locked_target')
    return dict(center_m=center.tolist(),visible_surface_centroid_m=np.median(p,axis=0).tolist(),top_points=len(p),observed_top_z_m=top,
        visible_edges=edges,hidden_dimensions_source='last_full_current_task_RGBD',
        source='current_visible_top_edges_and_locked_dimensions',occluded_edges_measured=False)

def measure(rgb,depth,snapshot,expected,axis,model,appearance):
    p=pixel_points(depth,snapshot);hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    d=abs(hsv[:,:,0].astype(float)-appearance['hue']);d=np.minimum(d,180-d)
    c=np.asarray(expected);radius=np.hypot(model['length_m'],model['width_m'])/2
    mask=(np.isfinite(depth)&(depth>0)&(d<=appearance['tolerance'])&(hsv[:,:,1]>45)&(hsv[:,:,2]>30)
          &(np.linalg.norm(p[:,:,:2]-c[:2],axis=2)<radius+.025)&(abs(p[:,:,2]-c[2])<.035))
    value=fit_points(p[mask],expected,axis,model)
    top=value['observed_top_z_m']
    return dict(value,class_name='cigarette',shape='flat_cuboid',height_m=model['height_m'],
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        grasp_point_base_m=[*value['center_m'][:2],top-model['height_m']+model['grip_above_bottom_m']])

def locked_patch(rgb,depth,snapshot,expected,axis,model,appearance):
    """Validate a locked near-contact pose; cannot authorize another approach.

    XY remains the last fully measured pose transformed by actual base TF.
    Current visible depth verifies the surface, and can independently prove lift.
    """
    p=pixel_points(depth,snapshot);c=np.asarray(expected,float);a=np.asarray(axis,float).copy()
    a[2]=0.;a/=np.linalg.norm(a);n=np.array([-a[1],a[0],0.]);v=p-c
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);d=abs(hsv[:,:,0].astype(float)-appearance['hue']);d=np.minimum(d,180-d)
    mask=(np.isfinite(depth)&(depth>0)&(d<=appearance['tolerance'])&(hsv[:,:,1]>45)&(hsv[:,:,2]>30)
          &(abs(v@a)<model['length_m']/2+.008)&(abs(v@n)<model['width_m']/2+.008)
          &(abs(v[:,:,2]-model['height_m']/2)<.012))
    points=p[mask]
    if len(points)<40:raise ValueError('locked_target_visible_depth_patch_missing')
    top=float(np.quantile(points[:,2],.98));points=points[abs(points[:,2]-top)<.0025]
    if len(points)<30:raise ValueError('locked_target_current_top_depth_missing')
    center=c.copy();center[2]=top-model['height_m']/2
    return dict(class_name='cigarette',shape='flat_cuboid',center_m=center.tolist(),height_m=model['height_m'],
        grasp_point_base_m=[*center[:2],top-model['height_m']+model['grip_above_bottom_m']],
        top_points=len(points),observed_top_z_m=top,visible_surface_centroid_m=np.median(points,axis=0).tolist(),
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],position_xy_is_prediction=True,
        current_visible_surface_depth_measured=True,additional_base_approach_authorized=False,
        source='locked_last_full_RGBD_XY_with_current_visible_coloured_top_depth')

def observe_known(rgb,depth,snapshot,expected,axis,model,appearance,*,allow_locked):
    """Full top -> visible edges -> locked near-contact patch, in one run."""
    from flat_box_geometry import measure_near
    try:return measure_near(rgb,depth,snapshot,expected,model,appearance)
    except ValueError as exc:
        if str(exc)!='flat_box_top_too_occluded_to_center':raise
    try:return measure(rgb,depth,snapshot,expected,axis,model,appearance)
    except ValueError:
        if not allow_locked:raise
    return locked_patch(rgb,depth,snapshot,expected,axis,model,appearance)

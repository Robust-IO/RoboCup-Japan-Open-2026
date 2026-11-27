"""Independent surrounding-depth plane hypothesis; never adjusts target center."""
import math
import cv2
import numpy as np
from hand_camera_ray import rotation
from rgbd_pixel_window import pixel_window


def fit_support(points,center,radius):
    points=np.asarray(points,float);center=np.asarray(center,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<150 or not np.isfinite(points).all():raise ValueError('insufficient_support_points')
    if center.shape!=(3,) or not np.isfinite(center).all() or not math.isfinite(radius) or not .015<=radius<=.065:raise ValueError('invalid_object_geometry')
    # Select height mode WITHOUT using object height/radius or known table height.
    bins=np.floor(points[:,2]/.005).astype(int);keys,counts=np.unique(bins,return_counts=True)
    peak=keys[np.argmax(counts)];initial=np.abs(points[:,2]-(peak+.5)*.005)<.012
    support=points[initial]
    if len(support)<150 or len(support)/len(points)<.45:raise ValueError('no_dominant_local_support')
    for _ in range(3):
        design=np.c_[support[:,:2],np.ones(len(support))]
        coefficients,_,rank,_=np.linalg.lstsq(design,support[:,2],rcond=None)
        if rank!=3:raise ValueError('degenerate_support')
        residual=np.abs(design@coefficients-support[:,2])
        support=support[residual<=.006]
        if len(support)<150:raise ValueError('noisy_support')
    coefficients=np.linalg.lstsq(np.c_[support[:,:2],np.ones(len(support))],support[:,2],rcond=None)[0]
    residual=np.abs(np.c_[support[:,:2],np.ones(len(support))]@coefficients-support[:,2])
    if np.quantile(residual,.95)>.004 or np.linalg.norm(coefficients[:2])>math.tan(math.radians(10)):raise ValueError('nonhorizontal_or_noisy_support')
    if min(np.ptp(support[:,:2],axis=0))<.04:raise ValueError('insufficient_support_extent')
    hull=cv2.convexHull(support[:,:2].astype(np.float32))
    if cv2.pointPolygonTest(hull,tuple(map(float,center[:2])),False)<0:raise ValueError('target_outside_support_hull')
    plane_z=float(np.r_[center[:2],1.]@coefficients)
    gap=float(center[2]-radius-plane_z)
    return dict(support_height_at_center_m=plane_z,plane_coefficients=coefficients.tolist(),
                plane_residual_p95_m=float(np.quantile(residual,.95)),support_points=len(support),
                support_fraction=len(support)/len(points),estimated_bottom_m=float(center[2]-radius),
                bottom_gap_m=gap,support_consistent=abs(gap)<=.015,
                center_adjusted=False,grasp_center_verified=False,motion_authorized=False,
                limitation='Surrounding depth supports a local plane hypothesis, not proof of contact or exact apple shape. Independent samples share camera calibration; systematic calibration bias is not ruled out.')


def check(depth,snapshot,detection,candidate):
    if candidate['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']:raise ValueError('candidate_depth_mismatch')
    if candidate['frame_id']!='base_link' or candidate['source']!='rgbd':raise ValueError('invalid_candidate_frame')
    x1,y1,x2,y2=detection['box'];h,w=depth.shape
    pad=max(25.,.6*max(x2-x1,y2-y1))
    window,u,v=pixel_window(depth.shape,x1-pad,y1-pad,x2+pad,y2+pad)
    depth=depth[window]
    ring=(u>=x1-pad)&(u<x2+pad)&(v>=y1-pad)&(v<y2+pad)&~((u>=x1-5)&(u<x2+5)&(v>=y1-5)&(v<y2+5))
    valid=ring&np.isfinite(depth)&(depth>0)&(depth<3)
    k=np.asarray(snapshot['rgb_k']).reshape(3,3);z=depth[valid];t=snapshot['base_from_rgb']
    if not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0:raise ValueError('invalid_intrinsics')
    points=np.c_[(u[valid]-k[0,2])*z/k[0,0],(v[valid]-k[1,2])*z/k[1,1],z]@rotation(t['quaternion']).T+t['translation']
    result=fit_support(points,candidate['center_m'],candidate['radius_m'])
    if 'visible_top_z_m' in candidate:
        top=candidate['visible_top_z_m'];height=top-result['support_height_at_center_m']
        result.update(visible_top_z_m=top,visible_height_above_support_m=height,
                      support_top_midpoint_z_m=(top+result['support_height_at_center_m'])/2,
                      center_minus_support_top_midpoint_m=candidate['center_m'][2]-(top+result['support_height_at_center_m'])/2)
    return result

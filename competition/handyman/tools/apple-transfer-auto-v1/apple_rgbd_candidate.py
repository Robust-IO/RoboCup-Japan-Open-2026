"""Apple-specific RGBD center candidate from detector box; no control/room claims."""
import numpy as np
from hand_camera_ray import rotation
from rgbd_pixel_window import pixel_window


def estimate(rgb, depth, snapshot, detection, *, geometry_proposal_only=False):
    minimum = .1 if geometry_proposal_only else .5
    if detection['name']!='apple' or not np.isfinite(detection['confidence']) or not minimum<=detection['confidence']<=1:
        raise ValueError('unconfirmed_apple_detection')
    if abs(snapshot['streams']['rgb']['stamp_ns']-snapshot['streams']['depth']['stamp_ns'])>50_000_000:
        raise ValueError('rgb_depth_unsynchronised')
    if rgb.shape[:2]!=depth.shape or rgb.shape[2]!=3:raise ValueError('shape_mismatch')
    box=np.asarray(detection['box'],float)
    if box.shape!=(4,) or not np.isfinite(box).all():raise ValueError('invalid_box')
    x1,y1,x2,y2=box;h,w=depth.shape
    if not 0<=x1<x2<=w or not 0<=y1<y2<=h:raise ValueError('invalid_box')
    if min(x2-x1,y2-y1)<15:raise ValueError('target_too_small')
    window,u,v=pixel_window(depth.shape,x1+2,y1+2,x2-2,y2-2)
    depth=depth[window];r,g,b=rgb[window].astype(float).transpose(2,0,1)
    mask=(u>=x1+2)&(u<x2-2)&(v>=y1+2)&(v<y2-2)&(r>80)&(r>1.35*g)&(r>1.2*b)&(b>.9*g)
    valid=mask&np.isfinite(depth)&(depth>0)&(depth<3)
    if mask.sum()<80 or valid.sum()/max(1,mask.sum())<.8:raise ValueError('insufficient_target_depth')
    z=depth[valid];med=np.median(z);valid &= abs(depth-med)<.08
    if valid.sum()<80:raise ValueError('insufficient_target_points')
    k=np.asarray(snapshot['rgb_k']).reshape(3,3)
    if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0:raise ValueError('invalid_intrinsics')
    t=snapshot['base_from_rgb'];z=depth[valid]
    pts=np.c_[(u[valid]-k[0,2])*z/k[0,0],(v[valid]-k[1,2])*z/k[1,1],z]@rotation(t['quaternion']).T+t['translation']
    if not np.isfinite(pts).all():raise ValueError('invalid_transform')
    origin=pts.mean(0);local=pts-origin
    sv=np.linalg.svd(local,compute_uv=False)
    if sv[-1]/sv[0]<.035:raise ValueError('insufficient_surface_curvature')
    q=np.linalg.lstsq(np.c_[2*local,np.ones(len(local))],(local*local).sum(1),rcond=None)[0]
    radius2=q[3]+q[:3]@q[:3]
    if not .015**2<=radius2<=.065**2:raise ValueError('implausible_apple_radius')
    center=q[:3]+origin;radius=float(np.sqrt(radius2))
    residual=float(np.quantile(abs(np.linalg.norm(pts-center,axis=1)-radius),.95))
    if residual>.012:raise ValueError('poor_center_fit')
    return dict(center_m=center.tolist(),radius_m=radius,fit_residual_p95_m=residual,
        identity_confirmed=False,geometry_proposal_only=geometry_proposal_only,
        points=len(pts),visible_surface_median_m=np.median(pts,axis=0).tolist(),visible_top_z_m=float(np.quantile(pts[:,2],.99)),
        frame_id='base_link',source='rgbd',position_semantics='estimated_grasp_center',
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],room_confirmed=False,
        ready_for_motion=False,requires=['fresh_multiframe_consistency','room_check','reachability'],
        limitation='Apple-only red surface plus approximate sphere. Fit residual is not total localization uncertainty.')

import cv2,numpy as np
from scipy.spatial import cKDTree
from round_body_geometry import grid
from hand_camera_ray import rotation
def register_colored_surface(rgb,depth,s,template_odom,grasp_odom,hue):
 tf=s['tracking_transforms']['odom_from_base_at_depth'];r=rotation(tf['quaternion']);t=np.asarray(tf['translation']);model=(np.asarray(template_odom)-t)@r;g=r.T@(np.asarray(grasp_odom)-t)
 p=grid(depth,s);lo=np.min(model,axis=0)-.025;hi=np.max(model,axis=0)+.025
 hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);dh=abs(hsv[:,:,0].astype(float)-hue);dh=np.minimum(dh,180-dh)
 mask=np.isfinite(depth)&(depth>.05)&(p>=lo).all(2)&(p<=hi).all(2)&(dh<12)&(hsv[:,:,1]>35)&(hsv[:,:,2]>30)
 q=p[mask]
 if len(q)<100:raise ValueError('current_target_registration_surface_missing')
 shift=np.zeros(3)
 for _ in range(15):
  tree=cKDTree(model+shift);dist,idx=tree.query(q);valid=dist<.018
  if valid.sum()<100:raise ValueError('current_target_registration_correspondences_missing')
  residual=q[valid]-(model[idx[valid]]+shift);step=np.median(residual,axis=0);shift+=step
  if np.linalg.norm(step)<.00015:break
 dist,idx=cKDTree(model+shift).query(q);valid=dist<.010;actual=q[valid];res=float(np.quantile(dist[valid],.9)) if valid.any() else 1
 if len(actual)<100 or np.linalg.norm(shift)>.035 or np.ptp(actual[:,1])<.015:raise ValueError('actual_target_surface_not_registered')
 return dict(center_m=(g+shift).tolist(),surface_shift_base_m=shift.tolist(),matched_surface_points=len(actual),surface_residual_p90_m=res,stamp_ns=s['streams']['depth']['stamp_ns'],position_source='current_visible_object_surface_registration',grasp_point_is_surface_anchored=True),r@shift

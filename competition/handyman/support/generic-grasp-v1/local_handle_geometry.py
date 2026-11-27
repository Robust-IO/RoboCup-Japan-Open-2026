"""Class-independent local side-pinch selection and observed surface registration."""
import numpy as np
from scipy.spatial import cKDTree
from round_body_geometry import grid
from hand_camera_ray import rotation

def plane_base(s,plane):
 c=np.asarray(plane);tf=s['tracking_transforms']['map_from_base_at_depth'];n=np.r_[-c[:2],1.];nb=rotation(tf['quaternion']).T@n
 return np.r_[-nb[:2]/nb[2],(c[2]-n@tf['translation'])/nb[2]]

def source_points(depth,s,box,plane):
 p=grid(depth,s);h=p[:,:,2]-(p[:,:,:2]@plane[:2]+plane[2]);x1,y1,x2,y2=map(int,box);roi=np.zeros(depth.shape,bool);roi[max(0,y1):y2,max(0,x1):x2]=True
 q=p[roi&np.isfinite(depth)&(depth>.05)&(h>.006)&(h<.3)]
 if len(q)<200:raise ValueError('insufficient_actual_target_surface')
 return q

def select_pinch(points,plane,envelope,orientation):
 front=float(np.quantile(points[:,0],.02));rows=[]
 # Select the first reachable pair of side surfaces; reject any gap > jaw width.
 for x in np.arange(front+.025,front+.050,.008):
  q=points[abs(points[:,0]-x)<.012];ys=np.sort(q[:,1]);cuts=np.r_[0,np.where(np.diff(ys)>.012)[0]+1,len(ys)]
  for lo,hi in zip(cuts[:-1],cuts[1:]):
   if hi-lo<30:continue
   low,high=np.quantile(ys[lo:hi],[.01,.99]);width=high-low
   if not .018<width<.075:continue
   y=float((low+high)/2);patch=q[(q[:,1]>=low)&(q[:,1]<=high)];top=float(np.quantile(patch[:,2],.95));bottom=float(np.r_[x,y,1.]@plane)
   # Full physical closing sweep, including table slope at the actual palm XY.
   palmxy=np.array([x-.062125866,y]);minimum=-np.inf
   for sample in envelope['samples']:
    for part in sample['parts']:
     if part.get('is_trigger'):continue
     v=np.asarray(part['vertices_palm_m'])@orientation.T
     minimum=max(minimum,float(np.max((v[:,:2]+palmxy)@plane[:2]+plane[2]-v[:,2]))+part['contact_offset_m']+envelope['environment_contact_offset_m']+envelope['uncertainty_bound_m']+.005)
   graspz=max(minimum+.003,bottom+(top-bottom)*.65)
   overlap=min(top,graspz+.0175)-max(bottom,graspz-.0175)
   if overlap<.012:continue
   rows.append(dict(center_m=[float(x),y,float(graspz)],width_m=float(width),top_m=top,bottom_m=bottom,minimum_palm_height_m=float(minimum),vertical_overlap_m=float(overlap),source_points=len(patch)))
 if not rows:raise ValueError('no_locally_graspable_region_with_table_clearance')
 # Prefer substantial side overlap, then modest insertion and little lateral travel.
 rows.sort(key=lambda v:(0 if v['vertical_overlap_m']>=.025 else 1,abs(v['center_m'][0]-(front+.033)),abs(v['center_m'][1])))
 return rows[0],rows

def register_surface(depth,s,template_odom,grasp_odom):
 tf=s['tracking_transforms']['odom_from_base_at_depth'];r=rotation(tf['quaternion']);t=np.asarray(tf['translation']);model=(np.asarray(template_odom)-t)@r;g=r.T@(np.asarray(grasp_odom)-t)
 p=grid(depth,s);lo=np.min(model,axis=0)-.025;hi=np.max(model,axis=0)+.025
 mask=np.isfinite(depth)&(depth>.05)&(p>=lo).all(2)&(p<=hi).all(2)
 q=p[mask]
 if len(q)<100:raise ValueError('current_target_registration_surface_missing')
 shift=np.zeros(3)
 for _ in range(15):
  tree=cKDTree(model+shift);dist,idx=tree.query(q);valid=dist<.018
  if valid.sum()<100:raise ValueError('current_target_registration_correspondences_missing')
  residual=q[valid]-(model[idx[valid]]+shift);step=np.median(residual,axis=0);shift+=step
  if np.linalg.norm(step)<.00015:break
 dist,idx=cKDTree(model+shift).query(q);valid=dist<.010;actual=q[valid];res=float(np.quantile(dist[valid],.9)) if valid.any() else 1
 if len(actual)<100 or np.linalg.norm(shift)>.035 or np.ptp(actual[:,1])<.075:raise ValueError('actual_target_surface_not_registered')
 return dict(center_m=(g+shift).tolist(),surface_shift_base_m=shift.tolist(),matched_surface_points=len(actual),surface_residual_p90_m=res,stamp_ns=s['streams']['depth']['stamp_ns'],position_source='current_visible_object_surface_registration',grasp_point_is_surface_anchored=True),r@shift

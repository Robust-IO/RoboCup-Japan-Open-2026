"""Name-independent measured round-body side grasp; original RGBD only."""
import cv2,numpy as np
from hand_camera_ray import rotation
from upright_cylinder_geometry import fit_side_circle

def grid(depth,s):
 k=np.array(s['rgb_k']).reshape(3,3);v,u=np.indices(depth.shape);z=depth.astype(float)
 xyz=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),axis=-1)
 t=s['base_from_rgb'];return xyz@rotation(t['quaternion']).T+t['translation']

def measure(rgb,depth,s,*,box=None,reference=None,expected=None,plane=None):
 p=grid(depth,s);valid=np.isfinite(depth)&(depth>.05)&(depth<3)
 hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
 if reference is None:
  x1,y1,x2,y2=map(int,box);roi=np.zeros(depth.shape,bool);roi[max(0,y1):min(depth.shape[0],y2),max(0,x1):min(depth.shape[1],x2)]=True
  coef=np.asarray(plane);h=p[:,:,2]-(p[:,:,:2]@coef[:2]+coef[2]);mask=valid&roi&(h>.008)&(h<.35)&(hsv[:,:,1]>50)
  if mask.sum()<100:raise ValueError('insufficient_measured_colored_body')
  hist=np.bincount(hsv[:,:,0][mask],minlength=180);hue=int(hist.argmax());dh=np.minimum(abs(hsv[:,:,0].astype(float)-hue),180-abs(hsv[:,:,0].astype(float)-hue));mask&=dh<12
  pts=p[mask];bottom=float(np.median(pts[:,:2],axis=0)@coef[:2]+coef[2]);top=float(np.quantile(pts[:,2],.99));height=top-bottom
 else:
  hue=reference['appearance_hue'];dh=np.minimum(abs(hsv[:,:,0].astype(float)-hue),180-abs(hsv[:,:,0].astype(float)-hue));e=np.asarray(expected)
  mask=valid&(dh<12)&(hsv[:,:,1]>50)&(np.linalg.norm(p[:,:,:2]-e[:2],axis=-1)<reference['max_radius_m']+.035)&(abs(p[:,:,2]-e[2])<reference['height_m']*.65+.025)
  pts=p[mask]
  if len(pts)<100:raise ValueError('current_body_surface_not_observed')
  height=reference['height_m'];top=float(np.quantile(pts[:,2],.99));bottom=top-height
 if not .03<height<.3:raise ValueError('invalid_measured_body_height')
 fits=[]
 for fraction in (.2,.35,.5,.65,.8,.9):
  z=bottom+height*fraction;band=pts[abs(pts[:,2]-z)<height*.06]
  try:c,r,error,arc=fit_side_circle(band[:,:2],[.008,.08]);fits.append(dict(center=c.tolist(),radius=r,z=z,error=error,fraction=fraction))
  except ValueError:pass
 if len(fits)<2:raise ValueError('round_body_shape_not_measured')
 centers=np.array([v['center'] for v in fits]);center=np.median(centers,axis=0)
 if np.max(np.linalg.norm(centers-center,axis=1))>.012:raise ValueError('body_axis_not_upright')
 choices=[v for v in fits if 2*v['radius']<.083 and v['z']-bottom>.025]
 if not choices:raise ValueError('no_measured_finger_clearance_candidate')
 chosen=min(choices,key=lambda v:abs(v['fraction']-.5)) if reference is None else min(fits,key=lambda v:abs(v['fraction']-reference['grasp_fraction']))
 grasp_z=chosen['z'] if reference is None else bottom+height*reference['grasp_fraction']
 return dict(center_m=[*center,grasp_z],height_m=height,bottom_z_m=bottom,top_z_m=top,max_radius_m=max(v['radius'] for v in fits),grasp_radius_m=chosen['radius'],grasp_fraction=chosen['fraction'],appearance_hue=hue,bands=fits,stamp_ns=s['streams']['depth']['stamp_ns'],strategy='measured_round_body_side_grasp',identity_from_geometry=False)

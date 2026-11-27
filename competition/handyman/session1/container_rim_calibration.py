"""Fit translated rectangular rim to a current measured cloud; no object truth."""
import numpy as np
from scipy.optimize import least_squares

def calibrate(points,opening):
    p=np.asarray(points,float);poly=np.asarray(opening['polygon_map_m'],float)
    low=poly.min(0);high=poly.max(0);height=float(opening['rim_height_m'])
    p=p[np.isfinite(p).all(1)]
    p=p[(abs(p[:,2]-height)<.025)&(p[:,:2]>=low-.18).all(1)&(p[:,:2]<=high+.18).all(1)]
    if len(p)<60:raise ValueError('current_bin_rim_not_observed')
    xy=p[:,:2]
    # Keep one sample per 5mm cell so a dense single visible face cannot dominate.
    _,ix=np.unique(np.floor(xy/.005).astype(int),axis=0,return_index=True);xy=xy[ix]
    def distances(delta):
        q=xy-delta
        return np.stack([np.hypot(q[:,0]-low[0],np.maximum(np.maximum(low[1]-q[:,1],q[:,1]-high[1]),0)),
          np.hypot(q[:,0]-high[0],np.maximum(np.maximum(low[1]-q[:,1],q[:,1]-high[1]),0)),
          np.hypot(q[:,1]-low[1],np.maximum(np.maximum(low[0]-q[:,0],q[:,0]-high[0]),0)),
          np.hypot(q[:,1]-high[1],np.maximum(np.maximum(low[0]-q[:,0],q[:,0]-high[0]),0))],axis=1)
    fits=[]
    for x in (-.12,0,.12):
      for y in (-.12,0,.12):
        fit=least_squares(lambda delta:distances(delta).min(1),[x,y],bounds=(-.18,.18),loss='soft_l1',f_scale=.008,max_nfev=80)
        ds=distances(fit.x);near=ds.min(1)<.012;side=ds.argmin(1)
        counts=[int(np.sum(near&(side==i))) for i in range(4)]
        if sum(c>=4 for c in counts)<3 or near.mean()<.65:continue
        fits.append((float(np.mean(ds.min(1)[near]**2)),fit.x,counts,float(near.mean())))
    if not fits:raise ValueError('current_bin_rim_edges_ambiguous')
    _,delta,counts,fraction=min(fits,key=lambda x:x[0])
    updated=dict(opening,polygon_map_m=(poly+delta).tolist(),source='current_rgbd_rim_translation')
    return updated,dict(translation_xy_m=delta.tolist(),edge_counts=counts,inlier_fraction=fraction,points=len(xy))

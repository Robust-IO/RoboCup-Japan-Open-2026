"""Observed center candidates without eroding by the whole object's radius."""
import numpy as np
from scipy.ndimage import distance_transform_edt
from session_table_support import GRID_M

def observed_centers(support,center,half,inset=.02):
    center=np.asarray(center,float);half=np.asarray(half,float)
    if not np.isfinite(inset) or inset<.005 or np.any(half<=inset):
        raise ValueError('invalid_center_inset')
    v,u=np.nonzero(support.mask)
    points=support.origin+np.c_[u+.5,v+.5]*GRID_M
    return points[(np.abs(points-center)<=half-inset).all(1)]

def free_space_clearance(support,points,plane):
    """Ranking only: radius of observed free tabletop around each cell.

    Unknown cells and visible objects bound the free region. This adds no new
    acceptance gate; the planner still checks reach, patch and swept corridor.
    """
    mask=support.mask.copy()
    points=np.asarray(points,float)
    height=points[:,2]-np.c_[points[:,:2],np.ones(len(points))]@np.asarray(plane)
    occupied=points[(height>.015)&np.isfinite(points).all(axis=1)]
    cells=np.floor((occupied[:,:2]-support.origin)/GRID_M).astype(int)
    valid=(cells>=0).all(axis=1)&(cells[:,0]<mask.shape[1])&(cells[:,1]<mask.shape[0])
    cells=cells[valid]
    mask[cells[:,1],cells[:,0]]=False
    return distance_transform_edt(np.pad(mask,1))[1:-1,1:-1]*GRID_M

def free_space_rank(clearance,distance,xy):
    # 2 cm bands avoid letting millimetre-level depth noise dominate travel.
    return (-int(np.floor((clearance+1e-9)/.02)),distance,float(xy[0]),float(xy[1]))

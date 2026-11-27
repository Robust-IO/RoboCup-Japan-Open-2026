"""Per-part clearance of the same full hand sweep used by the planner."""
import numpy as np

def hand_table_clearance(palm,rotation,plane,hand,hull=None):
    vertices=hand.sweep@rotation.T+palm
    gaps=vertices[:,:,2]-vertices[:,:,:2]@plane[:2]-plane[2]-hand.offset[:,None]
    if hull is not None:
        inside=(vertices[:,:,:2]@hull.equations[:,:2].T+hull.equations[:,2]<=0).all(2)
        gaps=np.where(inside,gaps,np.inf)
    return float(gaps.min()-hand.environment_offset-hand.uncertainty)

"""Replan side contact against the refreshed close-range surface before descent."""
from dataclasses import replace
import numpy as np
from scipy.spatial import cKDTree
from .planner import Scene,plan

def replan_side(target,points,plane,stamp,hand,policy,*,observed_target=None):
    target=np.asarray(target,float);points=np.asarray(points,float)
    # Preserve current colour/geometry correspondences, including residuals
    # between 6mm and the registration's 8mm acceptance distance. They remain
    # target collision geometry, not an unclassified environment obstacle.
    if observed_target is not None:
        observed=np.asarray(observed_target,float)
        if observed.ndim!=2 or observed.shape[1]!=3 or not np.isfinite(observed).all():
            raise ValueError('invalid_current_target_points')
        if len(observed):target=np.concatenate((target,observed),axis=0)
    # Only measured neighbours of the registered target are attributed to it.
    # All other close-range geometry remains an obstacle.
    distance=cKDTree(target).query(points)[0]
    environment=points[distance>.006]
    scene=Scene(target,environment,np.asarray(plane),stamp)
    constrained=replace(policy,allowed_strategies=('side',),feasible_budget=4,preferred_strategy='side',
                        rotation_search=False,neutral_wrist_only=True)
    return plan(scene,hand,constrained)

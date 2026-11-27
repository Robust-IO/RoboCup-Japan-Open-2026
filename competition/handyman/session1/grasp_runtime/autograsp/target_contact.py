"""Only measured finger-pad neighbourhoods may intentionally touch the target."""
import numpy as np
from scipy.spatial import cKDTree
from .tracking import outside_boxes

def permitted_target_contact(points,target,palm,orient,contacts):
    points=np.asarray(points);target=np.asarray(target)
    if not len(points) or not len(target):return np.zeros(len(points),bool)
    belongs=cKDTree(target).query(points)[0]<=.006
    # Five millimetres around actual exported pad surfaces, never the palm or
    # a class-wide bounding box. The rest of the open hand remains checked.
    pads=np.asarray(contacts)@orient.T+palm
    return belongs & ~outside_boxes(points,pads,margin=.005)

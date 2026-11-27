"""Confidence policy applies only with explicit task source constraints."""
import math

def confidence_passes(confidence,source_constrained=False):
    return math.isfinite(confidence) and (confidence>.5 if source_constrained else confidence>=.8)

def within_source(center,table,margin_m=.03):
    """Furniture association tolerates RGBD/map error; not placement clearance."""
    if not math.isfinite(margin_m) or margin_m<0:raise ValueError("invalid_source_margin")
    bounds=table['axis_aligned_bounds'];c=bounds['center'];s=bounds['size']
    return all(math.isfinite(center[i]) and abs(center[i]-c[axis])<=s[axis]/2+margin_m for i,axis in enumerate(('x','y')))


def identity_component(points,confidence):
    """Identity uses RGB confidence; connectivity never rejects measured depth.

    Keep finite measured points for downstream grasp geometry. Missing depth is
    still unavailable geometry, not a failed RGB identity classification.
    """
    import numpy as np
    from .tracking import UnobservedSurface
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=3:raise ValueError('invalid_identity_depth_shape')
    mask=np.isfinite(p).all(axis=1)
    if not mask.any():raise UnobservedSurface('no_measured_depth_for_grasp_localization')
    return mask,True


def source_association(center,table,confidence=0.):
    """High-confidence edge recovery; room membership is checked separately."""
    if within_source(center,table):return True
    return math.isfinite(confidence) and confidence>.85 and within_source(center,table,.15)

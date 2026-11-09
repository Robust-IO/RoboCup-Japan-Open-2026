"""Can approach positioning: align sideways while clear, then move straight in.

No fixed subdivision of the forward distance; each step uses new measured RGBD.
Fifteen cm is a positioning target before lateral motion, not a travel limit.
"""
import math

LATERAL_ALIGNMENT_TOLERANCE_M=.004


def approach_plan(error):
    if len(error)!=3 or not all(math.isfinite(x) for x in error):raise ValueError('invalid_can_contact_error')
    forward,side,height=map(float,error)
    if abs(height)>.020:raise ValueError('can_approach_height_not_aligned')
    if abs(side)>LATERAL_ALIGNMENT_TOLERANCE_M:
        if forward<.13:
            return dict(phase='clear_before_lateral_alignment',delta=[forward-.15,0.,0.])
        return dict(phase='lateral_alignment',delta=[0.,side,0.])
    return dict(phase='straight_contact_approach',delta=[forward,0.,0.])

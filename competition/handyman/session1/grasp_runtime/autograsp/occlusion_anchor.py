"""Use a same-attempt anchor only with geometric evidence of hand occlusion."""
import numpy as np
from .coordinator import AttemptFailure
from .recovery import Failure
from .tracking import visible_reference

MISSING={'too_few_observed_surface_points','insufficient_geometry_colour_correspondences',
         'insufficient_distinct_surface_correspondences','only_tiny_surface_fragment_visible'}

def allowed(reason,fresh,blocked_fraction):
    return fresh and reason in MISSING and np.isfinite(blocked_fraction) and blocked_fraction>=.35

def fallback(backend,exc):
    b=backend
    if exc.failure!=Failure.OCCLUDED or not getattr(b,'preextension_anchor_confirmed',False):raise exc
    item,m=b._observe_locked();s=item['snapshot'];R,t=b._tf(s);model=(b.template-t)@R
    palm=b.rt.palm_pose(s)
    from hand_camera_ray import rotation
    orient=rotation(palm['quaternion']);opening=float(s['joints']['hand_motor_joint'])
    boxes=[np.asarray(p['vertices_palm_m'])@orient.T+palm['translation']
           for p in b.physical.sample(opening)['parts'] if not p.get('is_trigger')]
    visible=visible_reference(model,s['base_from_rgb']['translation'],boxes)
    blocked=1.-float(np.mean(visible)) if len(visible) else 0.
    if not allowed(str(exc),True,blocked):raise exc
    m=dict(m,occlusion_anchor_authorized=True,predicted_hand_occlusion_fraction=blocked,
           depth_measured=False,position_source='same_attempt_preextension_anchor_under_hand_occlusion')
    b.record(dict(event='hand_occlusion_anchor_retained',reason=str(exc),measurement=m,
                  rotation_recovery_allowed=True,object_motion_ruled_out=False))
    return item,m

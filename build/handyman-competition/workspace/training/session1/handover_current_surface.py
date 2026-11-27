"""Current RGBD registration for held delivery; no unobserved-pose fallback."""
import numpy as np
from handover import held_center
from grasp_runtime.autograsp.tracking import register_surface,UnobservedSurface
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure

def locate(context,palm,rotation,points,colours,registration=register_surface):
    R=rotation(palm['quaternion']);t=np.asarray(palm['translation'])
    template=np.asarray(context['vertices_palm_m'])@R.T+t
    points=np.asarray(points);colours=np.asarray(colours)
    near=(points>=template.min(0)-.06).all(1)&(points<=template.max(0)+.06).all(1)
    try:fit=registration(template,np.asarray(context['surface_rgb']),points[near],colours[near],max_shift=.06)
    except UnobservedSurface as exc:raise AttemptFailure(Failure.OCCLUDED,'handover_current_surface:'+str(exc)) from exc
    return t+R@held_center(context)+fit.translation,dict(matched_points=len(fit.matched_points),
        residual_p90_m=fit.residual_p90_m,template_coverage=fit.template_coverage,
        current_surface_shift_m=fit.translation.tolist())

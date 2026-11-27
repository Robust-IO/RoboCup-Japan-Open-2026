"""Real RGBD alignment at every final side or overhead grasp pose, never a stale anchor."""
import numpy as np

def align_before_close(backend):
    b=backend
    was_locked=b.pose_locked
    def observe():
        from .contact_observation import observe_current
        item,measurement=observe_current(b)
        # Lock the newly observed environment, not the earlier distant snapshot.
        R,t=b._tf(item['snapshot']);points,_,_=b._cloud(item)
        b.locked_environment_world=points@R.T+t;b._path_cache=None
        return item,measurement
    from .coordinator import AttemptFailure
    from .recovery import Failure
    item,m=observe()
    for attempt in range(6):
        error=np.asarray(b.rt.contact_error(m['center_m'],item['snapshot']),float)
        fit=b._actual_contact_check(item['snapshot'])
        b.record(dict(event='preclose_camera_alignment',strategy=getattr(b,'candidate',{}).get('strategy'),attempt=attempt,error_base_m=error.tolist(),measurement=m,fit=fit))
        if fit['fits']:
            b.record(dict(event='preclose_camera_confirmed',corrections=attempt,measurement=m))
            return item,m
        if attempt==5:raise AttemptFailure(Failure.MISALIGNED,'preclose_alignment_not_converged:'+fit['reason'])
        dz=b.preclose_height_delta(error,item['snapshot'])
        moved=False
        if abs(dz)>.00001:
            goal=list(b.rt.state()['joints']);goal[0]+=dz
            if not 0<=goal[0]<=b.policy.max_lift_m-b.policy.lift_check_m:
                raise AttemptFailure(Failure.MISALIGNED,'preclose_height_outside_reach')
            b._arm(goal,'side-preclose-camera-height');moved=True
        if np.linalg.norm(error[:2])>.003:
            # Measured correction, capped at 3cm to avoid one noisy fit oversteering.
            delta=error[:2]*min(1.,.03/np.linalg.norm(error[:2]))
            b._base([float(delta[0]),float(delta[1]),0.],'side-preclose-camera-alignment');moved=True
        if not moved:raise AttemptFailure(Failure.MISALIGNED,fit['reason'])
        item,m=observe()

# Compatibility for callers/tests of the original side-only helper.
align_side_before_close=align_before_close

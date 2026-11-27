"""Recover a temporarily invisible support surface without exiting the task."""
from .coordinator import AttemptFailure
from .recovery import Failure

def observe(b,target,once):
    attempts=0
    heads=(-.15,-.30,-.50,-.65,0.)
    while True:
        b.check_task_cancelled()
        try:
            result=once(target)
            if attempts:b.record(dict(event='support_observation_recovered',attempts=attempts))
            return result
        except ValueError as exc:
            if str(exc)!='dominant_table_plane_not_established':raise
            reason=str(exc)
        except AttemptFailure as exc:
            if not attempts or exc.failure!=Failure.OCCLUDED:raise
            reason=str(exc)
        if b.holding_state()!='empty':
            raise AttemptFailure(Failure.HOLD_UNCERTAIN,'support_reobserve_requires_empty_hand')
        angle=heads[attempts%len(heads)];attempts+=1
        b.record(dict(event='support_observation_retry',reason=reason,attempt=attempts,
                      head_tilt=angle,task_retained=True))
        b.identity_confirmed=False;b.pose_locked=False
        b.static_precontact_anchor=False;b.preextension_anchor_confirmed=False
        b.check_task_cancelled()
        b.rt.face_source((0.,angle))
        b.rt.pump(.1)

"""Recover a current target view in place before descent or final closure."""
from .coordinator import AttemptFailure
from .recovery import Failure


def observe_current(b):
    locked=b.pose_locked;static=getattr(b,'static_precontact_anchor',False)
    b.pose_locked=False;b.static_precontact_anchor=False
    try:
        for index,head in enumerate((None,(0.,-.3),(0.,-.15),(0.,-.5))):
            check=getattr(b,'check_task_cancelled',lambda:None);check()
            if head is not None:
                b.record(dict(event='contact_view_recovery',attempt=index,head_goal=head,
                              base_motion_sent=False,stale_anchor_accepted=False))
                b.rt.face_source(head);check()
            try:
                item,m=b._observe_tracking()
                if m.get('depth_measured') is not True:
                    raise AttemptFailure(Failure.OCCLUDED,'contact_requires_current_target_depth')
                return item,m
            except AttemptFailure as exc:
                if exc.failure!=Failure.OCCLUDED:raise
                if index==3 or not callable(getattr(b.rt,'face_source',None)):raise
    finally:
        b.pose_locked=locked;b.static_precontact_anchor=static

"""Recenter incomplete geometry and recover exhausted views within the task."""
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure


def reacquire(stages,cancel):
    b=stages.grasp_backend
    from observation_head_views import source_head_views
    queue=[tuple(getattr(b,'identity_head_goal',(0.,0.))),*source_head_views(),(0.,-.5),(0.,-.7)]
    view_limit=len(queue)+3  # Allow fresh recenter hints without starving side views.
    visited=[];last=None
    while queue and len(visited)<view_limit:
        head=queue.pop(0)
        if any(max(abs(x-y) for x,y in zip(head,h))<.02 for h in visited):continue
        visited.append(head);stages.check(cancel);stages.runtime.face_source(head)
        b.identity_confirmed=False;b.pose_locked=False
        try:
            scene=b.observe_stopped(stages.target);stages.check(cancel)
            b.record(dict(event='grasp_staging_identity_reacquired',head=list(head),attempt=len(visited)))
            return scene
        except AttemptFailure as exc:
            if exc.failure!=Failure.OCCLUDED:raise
            last=exc
            hint=getattr(b,'identity_head_hint',None)
            if hint is not None:queue.insert(0,tuple(hint))
            b.record(dict(event='grasp_staging_head_retry',head=list(head),reason=str(exc),next_head=queue[0] if queue else None))
    if last is not None:raise last
    raise AttemptFailure(Failure.OCCLUDED,'staging_view_unavailable')


def prepare_with_recovery(stages,effect,cancel,prepare):
    b=stages.grasp_backend;attempt=0
    while True:
        stages.check(cancel)
        try:
            if attempt:
                stages.reacquire_after_staging(cancel)
            return prepare(stages,effect,cancel)
        except AttemptFailure as exc:
            if exc.failure not in (Failure.NO_CANDIDATE,Failure.OCCLUDED):raise
            if b.holding_state()!='empty':raise
            attempt+=1
            b.pending_grasp_scene=None
            b.grasp_attempt_cycle=max(2,getattr(b,'grasp_attempt_cycle',1)+1)
            b.identity_confirmed=False;b.pose_locked=False
            # Different first view, not immediate repetition of the exhausted view.
            b.identity_head_goal=(0.,(-.15,0.,-.3,-.5)[(attempt-1)%4])
            b.record(dict(event='grasp_staging_exhaustion_recovery',attempt=attempt,
                reason=str(exc),head_goal=b.identity_head_goal,task_aborted=False))
            stages.check(cancel)

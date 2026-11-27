"""Finite head sweep, then distinct reachable left/right views of the same source."""
import math
import numpy as np
from uuid import uuid4
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure
from perception_bridge import inside

from observation_head_views import source_head_views

def lateral_views(pose,polygon,distance=.20,center=None):
    left=(-math.sin(pose[2]),math.cos(pose[2]))
    rows=[[pose[0]+sign*distance*left[0],pose[1]+sign*distance*left[1],pose[2]]
            for sign in (1,-1) if inside(pose[0]+sign*distance*left[0],pose[1]+sign*distance*left[1],polygon)]

    if center is not None:
        for row in rows:row[2]=math.atan2(center['y']-row[1],center['x']-row[0])
    return rows

def shift_observation_view(stages,pose,cancel):
    """Short holonomic shift: one motion owner, then face furniture separately."""
    stages.check(cancel)
    state=stages.state(cancel);start=state['poses']['map'];selector=stages.candidate_selector
    distance=math.dist(start[:2],pose[:2])
    if distance>.6:return False
    # Candidate reachability alone does not prove the straight segment is clear.
    for q in np.linspace(start[:2],pose[:2],max(2,int(distance/.02)+2)):
        cell=selector.cell(q)
        if not (0<=cell[0]<selector.walk.shape[0] and 0<=cell[1]<selector.walk.shape[1]) or not selector.walk[cell]:
            stages.grasp_backend.record(dict(event='source_view_shift_skipped',reason='straight_segment_blocked',goal=pose))
            return False
    dx,dy=pose[0]-start[0],pose[1]-start[1];c,s=math.cos(start[2]),math.sin(start[2])
    stages.motion.stage='observe_target'
    stages.motion.adjust('source-view-shift-'+uuid4().hex,start,[c*dx+s*dy,-s*dx+c*dy,0.],state['joints'][:5],state['joints'][5])
    stages.check(cancel)
    actual=stages.state(cancel)['poses']['map']
    stages.grasp_backend.record(dict(event='source_view_shift_finished',goal=pose,actual=actual,heading_preserved_during_shift=True))
    return math.dist(actual[:2],start[:2])>=.04 or distance<.04

def _observe_cycle(stages,effect,cancel,origin=None):
    initial=origin if origin is not None else stages.state(cancel)['poses']['map'];backend=stages.grasp_backend
    polygon=stages.environment['rooms'][stages.source_room]['region']
    views=[None]
    selector=stages.candidate_selector
    if selector is not None:
        poses=lateral_views(initial,polygon,center=backend.table['axis_aligned_bounds']['center'])
        views+=selector.rank(initial,poses,stages.source_id)[0]
    last=None;last_result=None
    for vi,pose in enumerate(views):
        stages.check(cancel)
        if pose is not None:
            # Identity may refer to old image geometry; reacquire at every new view.
            backend.identity_confirmed=False;backend.pose_locked=False
            stages.runtime.prepare_navigation_from_current()
            if not shift_observation_view(stages,pose,cancel):continue
        stages.face_source_furniture(effect,cancel)
        heads=[None,*source_head_views()[1:]];seen=set()
        while heads:
            head=heads.pop(0)
            key=(0.,0.) if head is None else tuple(round(x,3) for x in head)
            if key in seen:continue
            seen.add(key)
            backend.identity_head_hint=None
            tilt=None if head is None else head[1]
            stages.check(cancel)
            try:
                result=stages._observe_target_once(effect,cancel,head=head)
                if result.get('confirmed') is True:return result
                last_result=result
            except AttemptFailure as exc:
                if exc.failure!=Failure.OCCLUDED:raise
                last=exc
                hint=getattr(backend,'identity_head_hint',None)
                if hint is not None and len(seen)<12:
                    heads.insert(0,hint)
                    backend.record(dict(event='source_edge_target_recenter',head=list(hint),identity_threshold_unchanged=True))
                backend.record(dict(event='source_view_miss',view_index=vi,head_pan=0. if head is None else head[0],head_tilt=tilt,reason=str(exc),next_action='next_distinct_view'))
    if last_result is not None:return last_result
    if last is not None:raise last
    raise ValueError('source_view_observation_unavailable')

def observe_furniture(stages,effect,cancel):
    cycle=0;origin=None
    while True:
        stages.check(cancel)
        try:
            if origin is None:origin=list(stages.state(cancel)['poses']['map'])
            result=_observe_cycle(stages,effect,cancel,origin=origin)
            if result.get('confirmed') is True or stages.candidate_selector is None:return result
            reason=result.get('reason','unconfirmed_source')
        except AttemptFailure as exc:
            if exc.failure!=Failure.OCCLUDED or stages.candidate_selector is None:raise
            reason=str(exc)
        cycle+=1
        stages.grasp_backend.record(dict(event='source_observation_retry',cycle=cycle,reason=reason,task_retained=True))
        stages.grasp_backend.identity_confirmed=False;stages.grasp_backend.pose_locked=False
        if cancel.wait(.5):stages.check(cancel)

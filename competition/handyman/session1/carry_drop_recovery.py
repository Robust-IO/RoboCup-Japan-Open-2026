"""Sessions 2/5/6: bounded floor search and regrasp; executor owns protocol."""
from copy import deepcopy
import math
import numpy as np

def enabled(stages):
    return (getattr(stages,'config',{}).get('session_profile',{}).get('session_number') in (2,5,6)
            and stages.config.get('policy',{}).get('carry_drop_recovery',False) is True
            and stages.context is not None and not getattr(stages,'drop_recovery_active',False)
            and stages.motion.stage=='navigate_destination')

def stable_pair(first,second,target):
    if second.get('depth_stamp_ns',0)<=first.get('depth_stamp_ns',0):return None
    if not first.get('floor_visible') or not second.get('floor_visible'):return None
    for a in first.get('floor_targets',[]):
        for b in second.get('floor_targets',[]):
            if a.get('target')==b.get('target')==target and np.linalg.norm(np.asarray(a['center_map_m'])-b['center_map_m'])<.06:
                return b
    return None

class DroppedTargetNotFound(RuntimeError):
    """Two complete floor scans failed; retire motion then publish Give_up."""

FLOOR_VIEWS=((0.,-.8),(-.6,-.8),(.6,-.8))

def search_dropped_target(stages,cancel,drop_pose):
    from container_floor_capture import ContainerFloorCapture
    from uuid import uuid4
    b=stages.grasp_backend;rt=stages.runtime
    marker={'id':'drop-search','axis_aligned_bounds':{'center':{'x':drop_pose[0],'y':drop_pose[1],'z':0},'size':{'x':.01,'y':.01,'z':.01}}}
    # One scanner keeps capture filenames unique across the two passes.
    scan=ContainerFloorCapture(b,b.worker,marker,target=stages.target,radius_m=1.8,
        map_point=stages.map_point,views=FLOOR_VIEWS)
    for pass_index in range(2):
        if pass_index:
            stages.check(cancel);rt.sample(profile='state');state=rt.state()
            old_stage=stages.motion.stage
            try:
                stages.motion.stage='withdraw'
                stages.motion.adjust('drop-search-retreat-'+uuid4().hex,state['poses']['map'],
                    [-.5,0.,0.],state['joints'][:5],state['joints'][5])
                stages.check(cancel)
            finally:stages.motion.stage=old_stage
            b.record(dict(event='carry_floor_search_retreat_complete',distance_m=.5))
        for view_index in range(len(FLOOR_VIEWS)):
            # Pair only frames from the same head view, never across turns.
            stages.check(cancel);first=scan()
            stages.check(cancel);second=scan()
            row=stable_pair(first,second,stages.target)
            b.record(dict(event='carry_floor_search',pass_number=pass_index+1,
                view_index=view_index,evidence_ref=second['evidence_ref'],
                target_found=row is not None,task_retained=True))
            if row is not None:return row
    stages.check(cancel)
    b.record(dict(event='carry_floor_search_exhausted',passes=2,next_action='Give_up'))
    raise DroppedTargetNotFound('floor_target_not_found_after_retreat')

def checkpoint(stages,cancel):
    from handover_reclose import confirm
    from grasp_runtime.autograsp.coordinator import AttemptFailure
    from grasp_runtime.autograsp.recovery import Failure
    b=stages.grasp_backend;rt=stages.runtime
    stages.check(cancel)
    # Actively reclose; a retained commanded angle alone cannot establish a hold.
    try:
        opening=confirm(rt,lambda:stages.check(cancel),b.record)
    except AttemptFailure as exc:
        if exc.failure!=Failure.EMPTY_CLOSE:raise
    else:
        stages.context['holding_opening_rad']=opening;b.opening=opening
        b.record(dict(event='carry_checkpoint_still_held',opening_rad=opening))
        return False
    b.record(dict(event='carry_drop_confirmed_empty',target=stages.target,protocol_messages_sent=False))
    rt.sample(profile='state');pose=rt.state()['poses']['map']
    room=stages.motion.room_at(pose)
    row=search_dropped_target(stages,cancel,pose)
    stages.drop_recovery_active=True
    old_context=stages.context
    saved=(stages.source_id,stages.source_room,b.table,stages.source_candidates,stages.source_goal)
    selector=stages.candidate_selector
    x,y,z=row['center_map_m']
    table={'id':'observed_drop_floor#'+room,'axis_aligned_bounds':{'center':{'x':x,'y':y,'z':-.01},'size':{'x':.6,'y':.6,'z':.02}}}
    previous_table=selector.tables.get(table['id'])
    selector.tables[table['id']]=table
    try:
        stages.source_id=table['id'];stages.source_room=room;b.table=table
        stages.source_candidates=[]
        for radius in (.65,.85):
            for angle in np.arange(0,2*math.pi,math.pi/4):
                gx,gy=x+radius*math.cos(angle),y+radius*math.sin(angle)
                cell=selector.cell([gx,gy])
                if selector.valid(cell) and selector.walk[cell]:
                    stages.source_candidates.append([gx,gy,math.atan2(y-gy,x-gx)])
        if not stages.source_candidates:raise ValueError('no_reachable_floor_grasp_candidate')
        stages.source_goal=stages.source_candidates[0]
        b.identity_confirmed=False;b.pose_locked=False
        goal=list(rt.state()['joints']);goal[5]=.65
        rt.move_joints(goal,'drop-recovery-open-empty-hand')
        b.holding='empty';b.evidence=None;b.baseline=None;b.lift_done=False
        serial=getattr(stages,'drop_recovery_serial',0)+1;stages.drop_recovery_serial=serial
        effect={'operation_id':f'drop-recovery-{serial}','stage':'grasp'}
        observation=stages.observe_target(effect,cancel)
        if observation.get('confirmed') is not True:raise ValueError('dropped_target_not_confirmed')
        result=stages.grasp(effect,cancel)
        if result.get('grasp_verified') is not True:raise ValueError('floor_regrasp_not_verified')
        # Retain the shared context object used by destination placement bindings.
        new=deepcopy(stages.context)
        new['carry_regrasp_generation']=serial
        new['task_source_table_id']=saved[0]
        old_context.clear();old_context.update(new);stages.context=old_context;b.held_context=old_context
        goal=list(rt.state()['joints']);goal[0]=max(goal[0],.25)
        b._arm(goal,'floor-recovery-raise-before-fold')
        goal[:5]=[.25,0.,0.,-math.pi/2,0.]
        b._arm(goal,'floor-recovery-fold')
        opening=confirm(rt,lambda:stages.check(cancel),b.record)
        old_context['holding_opening_rad']=opening;b.opening=opening
        b.record(dict(event='carry_drop_recovered',target=stages.target,protocol_messages_sent=False,destination=stages.destination_id))
        return True
    finally:
        stages.source_id,stages.source_room,b.table,stages.source_candidates,stages.source_goal=saved
        if previous_table is None:selector.tables.pop(table['id'],None)
        else:selector.tables[table['id']]=previous_table
        stages.drop_recovery_active=False

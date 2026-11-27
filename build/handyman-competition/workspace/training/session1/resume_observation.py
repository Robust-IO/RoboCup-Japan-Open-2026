"""Resume the received Session1 task before manipulation, without replaying handshake."""
import json,math
from pathlib import Path
from execution_plan import furniture_candidates

def restore(session,checkpoint,current_state,environment,source_navigation=None):
    path=Path(checkpoint)
    saved=json.loads(path.read_text())
    if saved.get('state')!='stopped':raise ValueError('stopped_checkpoint_required')
    if saved.get('task')!=session.snapshot()['task']:raise ValueError('checkpoint_task_mismatch')
    evidence=saved.get('evidence',{})
    if set(evidence)!={'navigate_room','navigate_source'}:raise ValueError('only_pregrasp_observation_resume_supported')
    if evidence['navigate_source'].get('arrived') is not True:raise ValueError('source_navigation_not_completed')
    protocol_log=path.parent/'resume-protocol.log'
    if not protocol_log.exists():protocol_log=path.parent.parent/'live.log'
    text=protocol_log.read_text()
    if 'Moderator: Instruction '+session.instruction not in text:raise ValueError('actual_received_instruction_required')
    if any(x in text for x in ('Moderator: Task_failed','Moderator: Task_succeeded','Moderator: Mission_complete','Robot: Object_grasped','Robot: Task_finished')):raise ValueError('checkpoint_round_already_advanced')
    nav=json.loads(Path(evidence['navigate_source']['evidence_ref']).read_text())
    if nav.get('status')!='arrived' or nav.get('nav_reset') is not True:raise ValueError('navigation_completion_missing')
    pose=current_state['poses']['map']
    allowed=furniture_candidates(environment,session.task.source_table)
    if nav.get('final_pose') is not None:allowed.append(nav['final_pose'])
    if source_navigation is not None:
        fresh_nav=json.loads(Path(source_navigation).read_text())
        if fresh_nav.get('status')!='arrived' or fresh_nav.get('nav_reset') is not True:raise ValueError('resume_view_navigation_not_completed')
        allowed.append(fresh_nav['final_pose'])
    needs_reposition=min(math.hypot(p[0]-pose[0],p[1]-pose[1]) for p in allowed)>.12
    if needs_reposition:
        tucked=[0.,0.,0.,-math.pi/2,0.]
        if source_navigation is None or max(abs(a-b) for a,b in zip(current_state['joints'][:5],tucked))>.05:
            raise ValueError('current_source_candidate_not_reached')
    # New operation IDs; no protocol readiness or Room_reached replay. The caller
    # must use this only in the same running Unity round, before any grasp.
    session.environment=session.task.layout;session.evidence=evidence
    stage='navigate_source' if needs_reposition else 'observe_target'
    session._request(stage)
    return {'checkpoint':str(path),'resumed_stage':stage,'handshake_replayed':False,
            'current_source_pose':pose,'requires_same_unity_round':True}

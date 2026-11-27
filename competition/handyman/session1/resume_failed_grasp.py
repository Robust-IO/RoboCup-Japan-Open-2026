"""Same-round empty-hand continuation after a failed grasp; reacquire geometry."""
import json,math
from pathlib import Path

def restore(session,assembled,checkpoint,current):
    path=Path(checkpoint);saved=json.loads(path.read_text())
    if saved.get('state')!='stopped' or saved.get('task')!=session.snapshot()['task']:
        raise ValueError('matching_stopped_task_required')
    evidence=saved.get('evidence',{})
    if set(evidence)!={'navigate_room','navigate_source','observe_target'}:
        raise ValueError('resume_only_before_successful_grasp')
    if current['joints'][5]<.60:raise ValueError('current_open_empty_hand_required')
    log=path.parent/'resume-protocol.log'
    if not log.exists():log=path.parent.parent/'live.log'
    if not log.exists():log=path.parent.with_suffix('.log')
    text=log.read_text()
    if 'Moderator: Instruction '+session.instruction not in text:raise ValueError('actual_instruction_required')
    if 'Robot: Object_grasped' in text or 'Robot: Task_finished' in text:raise ValueError('later_protocol_stage_already_sent')
    stages=assembled['stages'];table=stages.grasp_backend.table;center=table['axis_aligned_bounds']['center']
    journal=path.parent/'journal.jsonl'
    if journal.exists():
        events=[json.loads(line) for line in journal.read_text().splitlines()]
        failed=next((e['decision']['selected'] for e in reversed(events)
            if e.get('event')=='grasp_plan' and e.get('decision',{}).get('selected')),None)
        if failed is not None:
            stages.grasp_backend.resume_failed_pose=failed
            stages.grasp_backend.record(dict(event='resume_failed_orientation_reference',
                candidate_id=failed['id'],wrist_roll_rad=failed['wrist_roll_rad'],
                old_target_coordinates_reused=False))
    if math.dist(current['poses']['map'][:2],[center['x'],center['y']])>2.:raise ValueError('robot_no_longer_near_source')
    transfer=assembled['transfer'];shared=getattr(transfer,'shared',transfer)
    if abs(current['joints'][1])>.05:
        shared.withdraw_source()
        stages.runtime.prepare_navigation_from_current()
    stages.grasp_backend.identity_confirmed=False;stages.grasp_backend.pose_locked=False
    first_view=path.parent/'captures/identity-1/snapshot.json'
    if first_view.exists():
        joints=json.loads(first_view.read_text()).get('joints',{})
        if 'head_pan_joint' in joints and 'head_tilt_joint' in joints:
            stages.runtime.face_source((joints['head_pan_joint'],joints['head_tilt_joint']))
    session.environment=session.task.layout
    session.evidence={k:evidence[k] for k in ['navigate_room','navigate_source']}
    session._request('observe_target')
    return dict(checkpoint=str(path),resumed_stage='observe_target',handshake_replayed=False,
                source_navigation_replayed=False,old_target_geometry_reused=False,requires_same_unity_round=True)

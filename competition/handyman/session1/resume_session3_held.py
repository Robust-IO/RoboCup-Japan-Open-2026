"""Restore a verified Session3 grasp in the same Unity round; no replayed messages."""
import json
from pathlib import Path
from dataclasses import replace

def restore(session,stages,checkpoint,current):
    root=Path(checkpoint);saved=json.loads((root/'live-state.json').read_text())
    expected=replace(session.task,target=session.profile.get('corrected_target','dog_doll'),source_table=session.profile.get('corrected_source',{}).get('furniture_id','corner_sofa#lobby'))
    from dataclasses import asdict
    if saved.get('state')!='stopped' or saved.get('task')!=asdict(expected):raise ValueError('matching_session3_held_checkpoint_required')
    evidence=saved.get('evidence',{})
    if set(evidence)!={'navigate_room','navigate_source','observe_target','grasp'} or evidence['grasp'].get('grasp_verified') is not True:raise ValueError('resume_requires_completed_grasp_before_tuck')
    context=json.loads((root/'held-context.json').read_text())
    if context.get('target')!=expected.target or context.get('source_table')!=expected.source_table or context.get('lift_evidence',{}).get('verified') is not True:raise ValueError('verified_held_context_required')
    opening=current['joints'][5]
    if not .01<opening<.6 or abs(opening-context['holding_opening_rad'])>.03:raise ValueError('current_held_opening_changed')
    log=(root/'resume-protocol.log').read_text()
    if 'Robot: Object_grasped' not in log or 'Moderator: Corrected_instruction '+session.profile['corrected_instruction'] not in log:raise ValueError('actual_corrected_grasp_protocol_required')
    if any('Moderator: '+m in log for m in ('Task_failed','Task_succeeded','Mission_complete')) or 'Robot: Task_finished' in log:raise ValueError('previous_round_already_finished')
    if (root/'release-record.json').exists():raise ValueError('already_released')
    stages.context=context;session.task=expected;session.corrected=True;session.environment='LayoutA'
    session.preflight_ok=True;session.ready_requested=True;session.evidence=evidence;session._request('tuck')
    (stages.output/'held-context.json').write_text(json.dumps(context,indent=2))
    return dict(checkpoint=str(root),resumed_stage='tuck',grasp_replayed=False,grasp_message_replayed=False,
                next_check='fresh_hold_feedback',requires_same_running_unity_round=True)

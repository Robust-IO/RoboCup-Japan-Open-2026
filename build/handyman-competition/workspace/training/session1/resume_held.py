"""Resume a verified grasp in the same live round; never issue a second grasp."""
import json
import math
from pathlib import Path

def restore(session,stages,checkpoint,current):
    path=Path(checkpoint);saved=json.loads(path.read_text())
    if saved.get('state')!='stopped' or saved['task']!=session.snapshot()['task']:
        raise ValueError('matching_stopped_task_required')
    evidence=saved.get('evidence',{})
    base={'navigate_room','navigate_source','observe_target','grasp'}
    at_destination=set(evidence)==base|{'tuck','navigate_destination'}
    if set(evidence)!=base and not at_destination:
        raise ValueError('resume_only_after_grasp_or_before_unstarted_placement')
    reset_placement=saved.get('reason')=='ValueError:support_not_ready_without_valid_translation'
    if at_destination:
        if saved.get('reason') not in ('ValueError:no_clear_placement_patch_reachable_from_current_side',
                                      'ValueError:support_not_ready_without_valid_translation'):
            raise ValueError('resume_only_after_placement_planning_or_support_failure')
        captures=list((path.parent/'captures').glob('destination-*/snapshot.json'))
        before=max((json.loads(p.read_text()) for p in captures),key=lambda s:s['streams']['depth']['stamp_ns'])
        pose=before['tracking_transforms']['map_from_base_at_depth']['translation']
        if math.dist(current['poses']['map'][:2],pose[:2])>.10:
            raise ValueError('destination_pose_changed')
        if abs(current['joints'][5]-before['joints']['hand_motor_joint'])>.03:
            raise ValueError('held_opening_changed')
    report=json.loads((path.parent/'grasp-result.json').read_text())
    if report.get('status')!='grasp_verified':raise ValueError('verified_grasp_report_required')
    context=json.loads((path.parent/'held-context.json').read_text())
    if context.get('target')!='dog_doll' or context.get('source_table')!='white_side_table#living_room':raise ValueError('held_context_mismatch')
    if current['joints'][5]>=.60:raise ValueError('current_gripper_open')
    if (path.parent/'release-record.json').exists():raise ValueError('already_released')
    log=path.parent.with_suffix('.log')
    if not log.exists():log=path.parent.parent/'live.log'
    if 'Robot: Object_grasped' not in log.read_text():raise ValueError('previous_grasp_protocol_not_sent')
    stages.context=context
    if reset_placement:
        from hold_observer import HoldObserver
        from withdrawal_observer import WithdrawalObserver
        from table_withdrawal import TableWithdrawal
        from transfer_actions import tuck
        semantic=json.loads(Path(stages.config['semantic_map']).read_text())
        destination=next(t for t in semantic['destinations'] if t['id']==session.task.destination)
        hold=HoldObserver(stages.grasp_backend,context)
        hold('before-place')
        rt=stages.runtime;goal=rt.state()['joints'].copy();goal[0]=min(.69,goal[0]+.08)
        rt.move_joints(goal,'raise_held_before_destination_retreat')
        withdrawal=TableWithdrawal(rt,stages.motion,WithdrawalObserver(stages.grasp_backend,destination,lambda:context),
                                   label='held-destination-replan-withdraw')
        result=tuck(rt,observe_hold=hold,withdraw_from_source=withdrawal)
        (stages.output/'placement-reset-tuck.json').write_text(json.dumps(result))
    session.environment=session.task.layout;session.evidence=evidence;session._request('place' if at_destination else 'tuck')
    return dict(resumed_stage='place' if at_destination else 'tuck',checkpoint=str(path),grasp_replayed=False,
                next_check='current_visual_hold_before_withdrawal',requires_same_unity_round=True)

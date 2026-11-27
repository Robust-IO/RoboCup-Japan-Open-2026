"""Only independent verification after an acknowledged release; no release replay."""
import json
from pathlib import Path
from threading import Event

def restore(session,assembled,checkpoint,current):
    path=Path(checkpoint);saved=json.loads(path.read_text())
    if saved.get('state')!='stopped' or saved['task']!=session.snapshot()['task']:raise ValueError('matching_stopped_task_required')
    evidence=saved['evidence']
    if set(evidence)!={'navigate_room','navigate_source','observe_target','grasp','tuck','navigate_destination','place'}:raise ValueError('resume_only_after_release')
    placement=json.loads((path.parent/'placement-result.json').read_text())
    release=json.loads((path.parent/'release-record.json').read_text())
    if placement.get('release_sent') is not True or release.get('release_sent') is not True:raise ValueError('release_acknowledgement_required')
    if current['joints'][5]<.60:raise ValueError('current_open_hand_required')
    if 'Robot: Task_finished' in path.parent.with_suffix('.log').read_text():raise ValueError('completion_already_sent')
    context=json.loads(Path(evidence['grasp']['held_context_path']).read_text())
    stages=assembled['stages'];stages.context=context
    session.environment=session.task.layout;session.evidence=evidence;session._request('verify_placement')
    effect=session.effects[-1];transfer=assembled['transfer'];transfer.initialize(effect,Event())
    transfer.release=release;transfer.placement=placement;transfer.place_started=True
    return dict(resumed_stage='verify_placement',release_replayed=False,checkpoint=str(path),requires_same_unity_round=True)

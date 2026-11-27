"""Renew stale observations BEFORE one release; never retry a publication."""
import json
from pathlib import Path

EXPIRY_REASONS={'fresh_carry_images_required','fresh_current_carry_joint_feedback_required'}


def publish_once(acquire,publish,on_expired,*,max_observations=5):
    for index in range(max_observations):
        try:evidence=acquire()
        except ValueError as exc:
            if str(exc) not in EXPIRY_REASONS:raise
            on_expired(index,str(exc));continue
        # Deliberately outside the try: publication errors MUST NOT replay.
        return publish(evidence)
    raise ValueError('no_fresh_can_release_observation_no_command_sent')


def reviewed_plan(source,run_id,run_root):
    source=Path(source).resolve();run_root=Path(run_root).resolve()
    r=json.loads((source/'result.json').read_text())
    events=[json.loads(line) for line in (source/'journal.jsonl').read_text().splitlines()]
    if (source.parent!=run_root or r.get('context_run_id')!=run_id
            or r.get('status')!='stopped_requires_current_state_review'
            or r.get('stage')!='release_once_from_current_rgbd' or r.get('release_commands_sent')!=0
            or r.get('hand_released') is not False or r.get('prepare',{}).get('status')!='passed'
            or any(e.get('event')=='can_release_sent' for e in events)):
        raise ValueError('reviewed_pre_release_only_context_required')
    plan=json.loads((source/'placement-plan.json').read_text())
    if plan.get('class_name')!='canned_juice':raise ValueError('reviewed_can_plan_required')
    return plan

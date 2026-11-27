"""Recompute one recorded lift with corrected phase selection. No robot imports."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

p=argparse.ArgumentParser();p.add_argument('--code',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise ValueError('new_audit_output_required')
sys.path.insert(0,str(a.code))
from audit_preclose_identity_lift_follow import audit
from verify_apple_lift_sequence import verify
base=Path('/__HANDYMAN_PACKAGE__/runtime/apple-r14-reviewed-close-lift-20261018-01')
trial=base/'grasp/close-lift';result=json.loads((trial/'result.json').read_text())
recording=Path(result['observation_directory'])
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
paths=[base/'result.json',base/'grasp/result.json',trial/'result.json',
       trial/'visual-lift-verification.json',trial/'independent-lift-verification.json']
paths.extend(p for folder in recording.iterdir() if folder.is_dir() for p in folder.iterdir() if p.is_file())
before={str(p):sha(p) for p in paths}
independent=audit(trial,expected_recording=recording,expected_epoch=result['tracking_epoch'],expected_task_id=result['tracking_task_id'])
canonical=verify(trial)
if any(sha(Path(path))!=digest for path,digest in before.items()):raise ValueError('original_evidence_changed')
passed=independent['grasp_success_verified'] is True
value=dict(status='recorded_lift_settled_window_verified' if passed else 'recorded_lift_settled_window_inconclusive',
    independent=independent,canonical_detector_only=canonical,
    original_files_unchanged=True,original_hashes=before,
    original_trial_still_reports_success=result.get('effective_grasp_success_verified'),
    recorded_audit_not_new_motion_lease=True,robot_commands_sent=False,thresholds_widened=False,
    warning='Recorded verification of an actually executed single closure/lift. Not an uninterrupted whole transfer or new live permission.')
with a.output.open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False)
print(json.dumps(dict(status=value['status'],accepted_frames=independent['accepted_after_frames'],
    span_s=independent['after_span_s'],spread_m=independent['after_center_spread_m'],
    excluded_transition=independent['excluded_transition_captures'],rejected=independent['rejected'],
    original_files_unchanged=True,robot_commands_sent=False)),flush=True)
raise SystemExit(0 if passed else 1)

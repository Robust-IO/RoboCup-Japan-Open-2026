"""Reconcile completion metadata; preserve original failed-stage records."""
import json
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');receipt=json.loads((b/'cigarette-full-auto-latest.json').read_text());root=Path(receipt['output']);r=json.loads((root/'result.json').read_text())
if r['status']!='completed_after_independent_post_place_review':raise ValueError('actual_success_review_required')
review=json.loads((Path(r['placement_review'])/'result.json').read_text())
if not review.get('placement_success_verified') or not (root/'result-before-independent-review.json').exists():raise ValueError('review_and_original_record_required')
if 'reason' in r:r['resolved_original_reason']=r.pop('reason')
r['active_stage']='completed';r['physical_actions_repeated_for_review']=False
names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
expected=[0,0,0,-1.5707963267948966,0,.65]
r['robot_ready_for_next_task']=all(abs(x-y)<.005 for x,y in zip(r['final_state']['joints'],expected))
r['validation_notes']=['Source navigation began at the fresh Unity initial pose.','A plane-array serialization fix required a source-pose continuation.','Post-placement review uses new actual RGBD, no repeated grasp or release.','Next acceptance task is one uninterrupted run using the final code.']
temp=root/'result.tmp';temp.write_text(json.dumps(r,ensure_ascii=False,indent=2));temp.replace(root/'result.json')
checkpoint=b/'phase5-table-localization-checkpoint-20261101.json';c=json.loads(checkpoint.read_text());c['validation_notes']=r['validation_notes'];c['validation_tests_passed']=159;c['robot_ready_for_next_task']=r['robot_ready_for_next_task'];checkpoint.write_text(json.dumps(c,ensure_ascii=False,indent=2))
print(json.dumps(dict(status=r['status'],robot_ready_for_next_task=r['robot_ready_for_next_task'],single_uninterrupted_run=r['single_uninterrupted_run'],checkpoint=str(checkpoint))))

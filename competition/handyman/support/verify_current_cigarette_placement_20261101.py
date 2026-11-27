"""Independent current RGBD confirmation, with no motion or repeated release."""
import argparse,json,math,time,uuid
from pathlib import Path
import numpy as np
import rclpy
from apple_transfer_automation import RobotLock,write_json
from flat_box_place import TaskPlaceRuntime
from flat_box_place_geometry import placed_expected,verify_placed
from flat_box_geometry import measure_near
from can_destination_surface import measure
from canned_grasp_runtime import TOPICS

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();b=Path('/__HANDYMAN_PACKAGE__/runtime');place=b/'cigarette-place-20261101-07';root=a.output
prior=json.loads((place/'result.json').read_text())
if prior.get('release_commands_sent')!=1:raise ValueError('one_actual_prior_release_required')
held=json.loads((place/'held-context.json').read_text());plan=json.loads((place/'placement-plan.json').read_text());cfg=json.loads((place/'config.snapshot.json').read_text());table=next(x for x in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if x['id']==plan['destination_id'])
report=dict(status='checking_current_placement',robot_commands_sent=False,repeat_grasp_or_release=False,original_place=str(place));runtime=None
with RobotLock():
 root.mkdir();rclpy.init()
 try:
  runtime=TaskPlaceRuntime(root,'verify-current-placed-box-'+uuid.uuid4().hex,lambda _:None);rows=[]
  for index in range(12):
   sample=runtime.sample(processing_reserve_ns=0);s=sample['snapshot']
   if rows and s['streams']['depth']['stamp_ns']-rows[-1]['measurement']['stamp_ns']<200_000_000:runtime.pump(.1);continue
   m=measure_near(sample['rgb'],sample['depth'],s,placed_expected(s,plan),held['model'],held['appearance']);plane=measure(sample['depth'],s,table);capture=runtime.save(sample,f'actual-placed-{len(rows)}',m)
   rows.append(dict(snapshot=s,measurement=m,plane=plane,capture=str(capture)))
   if len(rows)>=2:break
  state=runtime.state();proof=verify_placed(rows,plan,state['joints'],cleanup_completed=True)
  publishers={t:runtime.node.count_publishers(t) for t in TOPICS}
  if any(publishers.values()):raise ValueError('unexpected_motion_owner_during_readonly_verification')
  report.update(status='current_placement_independently_verified',placement_success_verified=True,proof=proof,rows=rows,final_state=state,command_publishers=publishers,final_capture=rows[-1]['capture'])
 except Exception as exc:report.update(status='current_placement_not_verified',reason=str(exc))
 finally:
  if runtime:runtime.shutdown()
  rclpy.shutdown();write_json(root/'result.json',report)
if report.get('placement_success_verified'):
 receipt=json.loads((b/'cigarette-full-auto-latest.json').read_text());mission=Path(receipt['output']);old=json.loads((mission/'result.json').read_text());archive=mission/'result-before-independent-review.json'
 if not archive.exists():write_json(archive,old)
 old.update(status='completed_after_independent_post_place_review',physical_transfer_verified=True,placement_review=str(root),
  robot_ready_for_next_task=True,close_commands=1,release_commands=1,single_uninterrupted_run=False,
  source_navigation='cigarette-source-20261101-05',fresh_session_intake='cigarette-new-session-20261101-202048',
  task_finished_sent=False,original_failure_preserved=str(archive),final_state=report['final_state'])
 write_json(mission/'result.json',old)
 write_json(b/'phase5-table-localization-checkpoint-20261101.json',dict(status='requested_transfer_completed_after_code_fixes_and_readonly_review',
  task='卧室白桌香烟→客厅矮方桌',mission=str(mission),grasp='cigarette-grasp-20261101-15',carry='cigarette-carry-20261101-05',place=str(place),review=str(root),
  service=json.loads((b/'table-localizer-service.json').read_text()),configuration=str(b/'table_localization.json'),
  final_state=report['final_state'],old_target_context_must_not_be_replayed=True,
  next_fresh_session_launcher=str(b/'launch_fresh_cigarette_session_20261101.py'),navigation_parameters_changed=False,model_changed=False))
print(json.dumps(dict(status=report['status'],output=str(root),proof=report.get('proof'),reason=report.get('reason'))))

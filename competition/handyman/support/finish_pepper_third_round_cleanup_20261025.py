import json,math,time,sys,uuid
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
import numpy as np,yaml,rclpy
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import LiveBackend
from pepper_runtime import PepperRuntime
from pepper_geometry import measure_near
from pepper_place import expected_placed,verify_placed
from can_destination_surface import measure as measure_table
from task_outcome_policy import pose_matches,robot_ready
from inverse_table_place_plan import OFFSET,FLEX,WRIST
base=Path('/__HANDYMAN_PACKAGE__/runtime');source=base/'pepper-resume-forward-round-3-20261025-01';root=base/'pepper-third-round-open-cleanup-20261025-01'
old=json.loads((source/'result.json').read_text());meta=json.loads((source/'run-meta.json').read_text());cfg=json.loads((source/'config.snapshot.json').read_text());plan=json.loads((source/'place/placement-plan.json').read_text());held=json.loads((source/'held-context.json').read_text())
if (old.get('stage')!='withdraw_and_retract_open_hand' or old.get('release_commands_sent')!=1 or old.get('close_commands_sent')!=1
        or old.get('hand_released') is not True or old.get('grasp_success_verified') is not True
        or (source/'pepper-placement-retreat.json').exists() or meta['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()):
 raise ValueError('reviewed_one_release_and_no_previous_retreat_required')
result=dict(status='starting',source_run=str(source),close_commands_sent=0,release_commands_sent=0,task_finished_sent=False,old_close_commands_sent=1,old_release_commands_sent=1)
with RobotLock():
 root.mkdir(exist_ok=False);rt=None;backend=None;started=time.time_ns()
 with (root/'journal.jsonl').open('x') as stream:
  def record(event):
   stream.write(json.dumps(dict(time_ns=time.time_ns(),**event),allow_nan=False)+'\n');stream.flush()
  try:
   rclpy.init();rt=PepperRuntime(root,'pepper-empty-cleanup-'+uuid.uuid4().hex[:8],record)
   rt.sample();state=rt.state();rt.ownership()
   expected=[plan['palm_height_m']-OFFSET+.08,FLEX,0.,WRIST,0.,.65]
   if not pose_matches(state['joints'],expected):raise ValueError('current_open_already_raised_arm_required')
   result['before']=state;record(dict(event='previous_release_and_raise_verified_no_replay',current_state=state))
   backend=LiveBackend(cfg);backend.bind(root,'pepper-third-empty-cleanup',record);backend.stage='withdraw'
   backend.adjust('remaining-empty-retreat',state['poses']['map'],[-cfg['policy']['retreat_m'],0.,0.],state['joints'][:5],state['joints'][5])
   rt.tuck_open();state=rt.state()
   names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
   if not robot_ready(dict(zip(names,state['joints']))):raise ValueError('empty_tucked_cleanup_not_observed')
   table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==cfg['destination_id'])
   rows=[];last='no_current_placement'
   for i in range(9):
    sample=rt.sample();s=sample['snapshot']
    try:
     body,cap=expected_placed(s,plan);m=measure_near(sample['rgb'],sample['depth'],s,body,cap,held['model']);plane=measure_table(sample['depth'],s,table);rt.fresh_measurement(s)
    except ValueError as exc: last=str(exc);record(dict(event='new_final_frame_rejected_without_motion',reason=last));continue
    if rows and m['stamp_ns']-rows[-1]['measurement']['stamp_ns']<200_000_000:continue
    capture=rt.save(sample,'final-placed-'+str(i),m);rows.append(dict(measurement=m,snapshot=s,plane=plane,capture=str(capture)))
    if len(rows)==3:break
   if len(rows)!=3:raise ValueError('three_new_actual_placed_frames_required:'+last)
   state=rt.state();proof=verify_placed(rows,plan,state['joints'],cleanup_completed=True)
   write_json(root/'placement-verification.json',proof)
   result.update(status='third_forward_physical_transfer_verified_after_transport_cleanup',physical_transfer_verified=True,
     placement_success_verified=True,robot_ready_for_next_task=True,cleanup_completed=True,robot_commands_sent=True,
     uninterrupted_application_run=False,source_action_replayed=False,object_remains_held=False,
     last_fresh_robot_state=state,final_placement_rows=rows)
  except Exception as exc:result.update(status='stopped_requires_state_review',reason=str(exc))
  finally:
   if backend:backend.close()
   if rt:rt.shutdown()
   if rclpy.ok():rclpy.shutdown()
   result.update(completed_ns=time.time_ns(),elapsed_s=(time.time_ns()-started)/1e9);write_json(root/'result.json',result)
   print(json.dumps({k:v for k,v in result.items() if k not in ('final_placement_rows',)},allow_nan=False),flush=True)
if result.get('physical_transfer_verified') is not True:raise SystemExit(1)

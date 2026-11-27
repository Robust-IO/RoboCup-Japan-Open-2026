"""One finite lift of the existing closed contact; never reopen or reclose."""
import json,sys,time,uuid
from pathlib import Path
import numpy as np,rclpy
sys.path.insert(0,'/mnt/d/handyman-tools/apple-transfer-auto-v1')
from penguin_runtime import PenguinRuntime
from penguin_geometry import measure_near,lift_follow
from canned_grasp_geometry import transform_point
from apple_transfer_automation import RobotLock
from task_outcome_policy import pose_matches
base=Path('/mnt/d/handyman-training-runs');prior=base/'penguin-native-transfer-20261025-10'
p=json.loads((prior/'result.json').read_text());old_meta=json.loads((prior/'run-meta.json').read_text())
assert p['close_commands_sent']==1 and p['release_commands_sent']==0 and p['stage']=='close_once'
assert old_meta['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip()
before_path=Path(p['close']['before_capture']);bs=json.loads((before_path/'snapshot.json').read_text())
before=json.loads((before_path/'object-measurement.json').read_text());model=p['source_observations'][-1]['measurement']
holding=p['close']['closure_evidence']['observed_opening_rad']
root=base/'penguin-current-head-lift-test-20261025-01';run_id='penguin-head-lift-'+uuid.uuid4().hex[:10]
result=dict(class_name='toy_penguin',robot_commands_sent=False,close_commands_sent=0,release_commands_sent=0,
            predecessor_run=str(prior),prior_close_commands_sent=1,grasp_success_verified=False,
            placement_success_verified=False,base_commands_sent=False,rows=[])
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();events=[]
 runtime=PenguinRuntime(root,run_id,events.append)
 try:
  runtime.sample();state=runtime.state();runtime.ownership()
  assert pose_matches(state['joints'],p['last_fresh_robot_state']['joints'])
  assert .02<holding<=.62 and abs(state['joints'][5]-holding)<.03
  result['initial_state']=state
  runtime.lift_holding(holding);result['robot_commands_sent']=True
  anchor_palm=transform_point(bs,bs['palm_base_at_depth'])
  b0=transform_point(bs,before['center_m']);h0=transform_point(bs,before['cap_center_m']);last=0
  for index in range(6):
   sample=runtime.sample();s=sample['snapshot'];stamp=s['streams']['depth']['stamp_ns']
   if stamp-last<200_000_000:runtime.pump(.2);continue
   shift=transform_point(s,s['palm_base_at_depth'])-anchor_palm
   row=dict(stamp_ns=stamp)
   try:
    m=measure_near(sample['rgb'],sample['depth'],s,transform_point(s,b0+shift,inverse=True),transform_point(s,h0+shift,inverse=True),model)
    proof=lift_follow(before,bs,m,s,holding);row.update(measurement=m,proof=proof,verified=True)
   except ValueError as exc:m=None;row.update(verified=False,reason=str(exc))
   row['capture']=str(runtime.save(sample,'after-lift-'+str(index),m));result['rows'].append(row);last=stamp
   runtime.pump(.3)
  good=[r for r in result['rows'] if r['verified']]
  result.update(grasp_success_verified=len(good)>=3,final_state=runtime.state(),
                status='current_head_grasp_lift_verified' if len(good)>=3 else 'lift_executed_requires_visual_review',
                object_may_remain_held=True)
 except Exception as exc:result.update(status='stopped_requires_actual_state_review',reason=str(exc))
 finally:
  result['events']=events;result['completed_ns']=time.time_ns()
  runtime.shutdown();rclpy.shutdown()
  (root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
  (root/'config.snapshot.json').write_bytes((prior/'config.snapshot.json').read_bytes())
  (root/'run-meta.json').write_text(json.dumps(dict(run_id=run_id,run_root=str(root),boot_id=old_meta['boot_id'],
                                                object='toy_penguin',task_sha256=p['task_sha256']),indent=2))
  print(json.dumps({k:v for k,v in result.items() if k!='events'}),flush=True)

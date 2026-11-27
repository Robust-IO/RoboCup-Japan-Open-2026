"""Review the last no-close stop and fold its EMPTY open arm once."""
import json,sys,time
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
import rclpy
from apple_transfer_automation import RobotLock,write_json
from pepper_runtime import PepperRuntime
from task_outcome_policy import pose_matches,robot_ready
base=Path('/__HANDYMAN_PACKAGE__/runtime')
old=base/'pepper-automatic-rounds-20261024-03/legs/00-initial-reset'
previous=json.loads((old/'result.json').read_text());meta=json.loads((old/'run-meta.json').read_text())
if (previous['stage']!='align_and_approach_wide_body' or previous['close_commands_sent']
        or previous['release_commands_sent'] or previous['approach_commands_sent']
        or meta['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()):
    raise ValueError('reviewed_empty_no_base_approach_stop_required')
root=base/'pepper-empty-source-fold-20261024-01';root.mkdir(exist_ok=False)
report=dict(status='starting',gripper_commands_sent=0,base_commands_sent=0,human_object_reposition_required=False)
rclpy.init();runtime=None
with RobotLock(),(root/'journal.jsonl').open('x') as stream:
    def record(event):stream.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');stream.flush()
    try:
        runtime=PepperRuntime(root,'pepper-empty-source-fold',record);runtime.sample();state=runtime.state();runtime.ownership()
        if not pose_matches(state['joints'],previous['last_fresh_robot_state']['joints']) or abs(state['joints'][5]-.65)>.005:
            raise ValueError('actual_empty_open_pregrasp_pose_changed')
        runtime.tuck_open();state=runtime.state();names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        if not robot_ready(dict(zip(names,state['joints']))):raise ValueError('empty_tucked_pose_not_confirmed')
        report.update(status='empty_open_arm_folded_and_lowered',last_fresh_robot_state=state)
    except Exception as exc:report.update(status='stopped_requires_actual_state_review',reason=str(exc))
    finally:
        if runtime:
            try:report['last_fresh_robot_state']=runtime.state()
            except Exception:pass
            runtime.shutdown()
        rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if report['status']!='empty_open_arm_folded_and_lowered':raise SystemExit(1)

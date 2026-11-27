"""Read-only live capture test: no motion, gripper, or task messages."""
from competition_paths import is_run_path
import argparse,json,time,uuid
from pathlib import Path
from apple_transfer_automation import RobotLock,write_json
from flat_box_place import TaskPlaceRuntime
from canned_grasp_runtime import TOPICS
import rclpy
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
root=args.output
if not is_run_path(root.resolve()):raise ValueError('D_output_required')
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();runtime=TaskPlaceRuntime(root,'stop-observe-probe-'+uuid.uuid4().hex,lambda _:None)
 report=dict(status='starting',robot_commands_sent=False,physical_motion_performed=False)
 try:
  first=runtime.sample(processing_reserve_ns=0);runtime.ownership();before=runtime.state()
  original_stamp=first['snapshot']['streams']['depth']['stamp_ns'];marker=first['snapshot']['stop_observe_act'].copy()
  runtime.save(first,'stopped-capture');time.sleep(1.2)
  proof=runtime.fresh_measurement(first['snapshot'])
  if proof['oldest_image_age_s']<=.8 or first['snapshot']['streams']['depth']['stamp_ns']!=original_stamp:raise ValueError('long_compute_or_original_timestamp_test_failed')
  runtime.invalidate_observations()
  try:runtime.fresh_measurement(first['snapshot'])
  except ValueError as exc:invalidated=str(exc)
  else:raise ValueError('old_capture_survived_simulated_command_generation_change')
  second=runtime.sample(processing_reserve_ns=0);second_proof=runtime.fresh_measurement(second['snapshot']);runtime.save(second,'new-capture-after-invalidation')
  if second['snapshot']['streams']['depth']['stamp_ns']<=original_stamp:raise ValueError('fresh_capture_not_newer')
  runtime.ownership()
  report.update(status='read_only_live_probe_passed',first_capture=marker,long_compute_proof=proof,
   original_timestamp_unchanged=True,simulated_command_generation_change=True,old_capture_rejection=invalidated,
   second_capture=second['snapshot']['stop_observe_act'],second_capture_proof=second_proof,
   initial_state=before,final_state=runtime.state(),command_publishers={t:runtime.node.count_publishers(t) for t in TOPICS})
 except Exception as exc:report.update(status='probe_failed_no_robot_commands',reason=str(exc))
 finally:
  runtime.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report))

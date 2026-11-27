"""Read-only fresh state before executing a new room-specified task."""
from competition_paths import is_run_path
import argparse,json,time,uuid
from pathlib import Path
import rclpy
from pepper_runtime import PepperRuntime
from apple_transfer_automation import RobotLock,write_json
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--run',action='store_true');a=p.parse_args()
if not is_run_path(a.output.resolve()):raise ValueError('D_output_required')
root=a.output;report=dict(robot_commands_sent=False,task=dict(object='cigarette',source_room='bedroom',destination='square_low_table'))
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();runtime=PepperRuntime(root,'cigarette-task-readonly-'+uuid.uuid4().hex,lambda _:None)
 try:
  sample=runtime.sample(processing_reserve_ns=0);report.update(status='fresh_state_observed',capture=str(runtime.save(sample,'current')),state=runtime.state())
  from canned_grasp_runtime import TOPICS
  report['command_topics']={t:dict(publishers=runtime.node.count_publishers(t),subscribers=runtime.node.count_subscribers(t),
   endpoints=[dict(node=x.node_name,namespace=x.node_namespace,topic_type=x.topic_type,gid=list(x.endpoint_gid)) for x in runtime.node.get_subscriptions_info_by_topic(t)]) for t in TOPICS}
 except Exception as e:report.update(status='live_state_unavailable',reason=str(e),received_streams=list(runtime.received))
 finally:
  now=runtime.node.get_clock().now().nanoseconds
  report['stream_ages_s']={k:(now-runtime.stamp(m))/1e9 for k,m in runtime.received.items() if hasattr(m,'header')}
  report['history_counts']={k:len(v) for k,v in runtime.history.items()}
  runtime.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report))

"""Fresh-session localization and intake, then automatic source/grasp/carry/place."""
import argparse,json,subprocess,time,uuid
from pathlib import Path
import rclpy
from apple_transfer_automation import RobotLock,write_json
from flat_box_place import TaskPlaceRuntime
from table_localization_client import localize
from canned_grasp_runtime import TOPICS

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=a.output;b=Path('/__HANDYMAN_PACKAGE__/runtime');runtime=None
report=dict(status='starting',robot_commands_sent=False,fresh_live_intake=True,new_unity_session_assumed=False,old_target_reference_reused=False)
with RobotLock():
 root.mkdir();rclpy.init()
 try:
  runtime=TaskPlaceRuntime(root,'fresh-unity-session-'+uuid.uuid4().hex,lambda _:None)
  runtime.sample(processing_reserve_ns=0);runtime.ownership();report['before_localization']=runtime.state()
  report['localization']=localize(runtime,root,'session-start')
  sample=runtime.sample(processing_reserve_ns=0);capture=runtime.save(sample,'fresh-session')
  from current_task_entry import inspect_entry
  report['entry']=inspect_entry(runtime,b)
  report['old_target_reference_reused']=report['entry']['mode']=='continue_held'
  report['held_continuation_requires_live_confirmation']=True
  topics={t:dict(publishers=runtime.node.count_publishers(t),subscribers=runtime.node.count_subscribers(t)) for t in TOPICS}
  if any(v['publishers'] or v['subscribers']!=1 for v in topics.values()):raise ValueError('new_session_control_channels_not_exclusive')
  report.update(status='fresh_state_observed',state=runtime.state(),command_topics=topics,capture=str(capture))
 except Exception as exc:report.update(status='new_session_initialization_failed',reason=str(exc))
 finally:
  if runtime:runtime.shutdown()
  rclpy.shutdown();write_json(root/'result.json',report)
if report['status']=='fresh_state_observed':
 name='cigarette-full-auto-'+time.strftime('%Y%m%d-%H%M%S');out=b/name
 with (b/(name+'-console.log')).open('xb') as stream:child=subprocess.Popen(['/usr/bin/python3',str(b/'run_cigarette_transfer_20261101.py'),'--output',str(out),'--readiness-run',str(root)],stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
 receipt=dict(pid=child.pid,output=str(out),fresh_session_intake=str(root));write_json(b/'cigarette-full-auto-latest.json',receipt);print(json.dumps(receipt))
else:print(json.dumps(report))

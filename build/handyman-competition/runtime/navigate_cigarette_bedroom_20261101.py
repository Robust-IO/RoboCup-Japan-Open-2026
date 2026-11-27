"""Enter the requested room's main search point, no assumed object location."""
from competition_paths import is_run_path
import argparse,json,math,time,uuid,signal
from pathlib import Path
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import LiveBackend
from pepper_runtime import PepperRuntime
from apple_worker_client import Worker
from task_outcome_policy import pose_matches
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--run',action='store_true');args=p.parse_args()
if not args.run:raise ValueError('explicit_run_required')
if not is_run_path(args.output.resolve()):raise ValueError('D_output_required')
root=args.output;cfg=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());cfg.update(joint_feedback_max_age_s=.6,navigation_joint_feedback_max_age_s=.85,navigation_feedback_recovery=True)
report=dict(status='starting',task=dict(object='cigarette',source_room='bedroom',destination='square_low_table'),close_commands_sent=0,release_commands_sent=0,task_finished_sent=False,object_coordinates_from_unity_used=False)
runtime=backend=worker=None
def stop(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,stop)
with RobotLock():
 root.mkdir(exist_ok=False);started=time.time_ns()
 with (root/'journal.jsonl').open('x') as log:
  def record(e):log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
  def stage(s):report['stage']=s;write_json(root/'result.json',report)
  try:
   import rclpy
   rclpy.init();epoch='cigarette-bedroom-task-'+uuid.uuid4().hex;runtime=PepperRuntime(root,epoch,record);backend=LiveBackend(cfg);backend.bind(root,epoch,record)
   runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership();report['initial_state']=state
   if not pose_matches(state['joints'],[0,0,0,-math.pi/2,0,0]):raise ValueError('fresh_initial_empty_tucked_pose_required')
   report['available_spin_services']=[x for x in runtime.node.get_service_names_and_types() if 'spin' in x[0]]
   source=backend.room_at(state['poses']['map']);point=backend.environment['rooms']['bedroom']['search_points'][0]
   goal=[point[k] for k in ('x','y','yaw')];report['room_main_search_goal']=goal
   worker=Worker(cfg['model_python'],cfg['registry'],root/'model.json',imgsz=640)
   stage('navigate_to_bedroom_main_search_point');backend.stage='navigate_source'
   backend.navigate(source,'bedroom',goal,state['joints'][5],'bedroom-entry-navigation')
   runtime.sample(processing_reserve_ns=0)
   if backend.room_at(runtime.state()['poses']['map'])!='bedroom':raise ValueError('bedroom_entry_not_verified')
   stage('observe_bedroom_main_direction');runtime.face_source((0.,-.35));report['frames']=[]
   for index in range(3):
    sample=runtime.sample(processing_reserve_ns=0);capture=runtime.save(sample,'main-direction-'+str(index));result=worker.infer(capture,pump=runtime.pump);write_json(capture/'detections.json',result)
    report['frames'].append(dict(capture=str(capture),inference=result));write_json(root/'result.json',report)
   report.update(status='bedroom_entered_first_direction_observed',last_robot_state=runtime.state())
  except (Exception,KeyboardInterrupt) as e:
   report.update(status='stopped_requires_actual_state_review',reason=str(e) or 'interrupted')
   if runtime:
    try:report['last_robot_state']=runtime.state()
    except Exception:pass
  finally:
   if worker:worker.close()
   if backend:backend.close()
   if runtime:runtime.shutdown();rclpy.shutdown()
   report['elapsed_s']=(time.time_ns()-started)/1e9;write_json(root/'result.json',report);print(json.dumps(report))

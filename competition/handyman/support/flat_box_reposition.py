"""Hold-preserving navigation to another exported table side for observation."""
from competition_paths import is_run_path
import argparse,json,math,os,signal,time,uuid
from pathlib import Path
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import LiveBackend
from horizontal_can_hand_trial import HandCanRuntime
from flat_box_transport import from_grasp,silhouette
from apple_transfer_auto_plan import footprint
from task_outcome_policy import pose_matches

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True);p.add_argument('--grasp',type=Path,required=True);args=p.parse_args()
 if not args.run:print(json.dumps(dict(robot_commands_sent=False)));return
 base=Path('/__HANDYMAN_PACKAGE__/runtime')
 if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not is_run_path(args.output.resolve()) or not is_run_path(args.grasp.resolve()):raise ValueError('domain71_D_required')
 held,prior,reference=from_grasp(args.grasp);holding=held['holding_opening_rad']
 cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());cfg.update(joint_feedback_max_age_s=.6,navigation_joint_feedback_max_age_s=.85,navigation_feedback_recovery=True)
 table=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==cfg['destination_id']);c,h=footprint(table)
 goal=[float(c[0]+h[0]+.85),float(c[1]),math.pi]
 root=args.output;runtime=backend=None;report=dict(status='starting',class_name='cigarette',close_commands_sent=0,release_commands_sent=0,carry_destination_verified=False,object_remains_held=True,alternative_table_side='east',observation_goal=goal,navigation_parameters_changed=False,verification_rows=[])
 def stop(*_):raise KeyboardInterrupt()
 signal.signal(signal.SIGTERM,stop)
 with RobotLock():
  root.mkdir(exist_ok=False);started=time.time_ns()
  with (root/'journal.jsonl').open('x') as log:
   def record(e):log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
   def stage(s):report['stage']=s;write_json(root/'result.json',report)
   try:
    import rclpy
    rclpy.init();epoch='box-other-side-'+uuid.uuid4().hex;runtime=HandCanRuntime(root,epoch,record);backend=LiveBackend(cfg);backend.bind(root,epoch,record)
    runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
    if not pose_matches(state['joints'],prior['final_state']['joints']):raise ValueError('current_verified_flat_box_hold_required')
    write_json(root/'held-context.json',held);write_json(root/'config.snapshot.json',cfg)
    def check(label):
     sample=runtime.sample(processing_reserve_ns=200_000_000)
     if abs(runtime.state()['joints'][5]-holding)>.03:raise ValueError('holding_opening_changed')
     proof=silhouette(reference,sample['hand'],held['appearance']);runtime.fresh_measurement(sample['snapshot']);path=runtime.save(sample,label)
     report['verification_rows'].append(dict(capture=str(path),proof=proof));write_json(root/'result.json',report)
    check('current-held');stage('fold_and_tuck_for_another_table_side')
    q=runtime.state()['joints'];q[1]=0.;q[3]=-math.pi/2;runtime.move_joints(q,'fold_preserving_box_grip');check('folded-held')
    q=runtime.state()['joints'];q[0]=0.;runtime.move_joints(q,'lower_tucked_box');check('tucked-held');runtime.face_source((0.,-.5))
    stage('navigate_around_table_with_existing_navigation');backend.stage='navigate_destination'
    backend.navigate('living_room','living_room',goal,holding,'alternative-side-navigation',remaining_waypoints=[])
    check('other-side-held');report.update(status='flat_box_alternative_side_hold_verified',carry_destination_verified=True,final_state=runtime.state())
   except (Exception,KeyboardInterrupt) as e:
    report.update(status='stopped_requires_actual_state_review',reason=str(e) or 'interrupted')
    if runtime:
     try:report['last_robot_state']=runtime.state()
     except Exception:pass
   finally:
    if backend:backend.close()
    if runtime:runtime.shutdown();rclpy.shutdown()
    report['elapsed_s']=(time.time_ns()-started)/1e9;write_json(root/'result.json',report);print(json.dumps(report))
if __name__=='__main__':main()

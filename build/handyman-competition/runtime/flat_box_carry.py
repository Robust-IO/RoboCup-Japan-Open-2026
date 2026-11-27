from competition_paths import is_run_path
import argparse,json,math,os,signal,time,uuid
from pathlib import Path
import cv2
import numpy as np
from apple_transfer_automation import RobotLock,write_json
from outcome_transfer_runtime import OutcomeBackend as LiveBackend,OutcomeRuntime
from outcome_transfer_policy import snapshot as policy_snapshot,holding_feedback,hold_assessment,HoldEvidence
from horizontal_can_hand_trial import HandCanRuntime
from flat_box_transport import silhouette,shape,palm_pose,from_grasp
from hand_camera_ray import rotation
from task_outcome_policy import pose_matches
from head_joint_feedback import TaskHeadFeedback
class TaskCarryRuntime(OutcomeRuntime,TaskHeadFeedback,HandCanRuntime):
 default_observation_profile="hand"

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True);p.add_argument('--grasp',type=Path,required=True);p.add_argument('--destination-id');args=p.parse_args()
 if not args.run:print(json.dumps(dict(robot_commands_sent=False)));return
 base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');grasp=args.grasp
 if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not is_run_path(args.output.resolve()):raise ValueError('domain71_D_required')
 if not is_run_path(grasp.resolve()):raise ValueError('D_grasp_required')
 held,prior,reference=from_grasp(grasp)
 model=held['model'];appearance=held['appearance'];holding=held['holding_opening_rad']
 cfg=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());cfg.update(navigation_joint_feedback_max_age_s=.85,navigation_feedback_recovery=True)
 if args.destination_id:cfg['destination_id']=args.destination_id
 destination=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==cfg['destination_id'])
 b=destination['axis_aligned_bounds'];cfg['destination_staging_goal']=[b['center']['x']+b['size']['x']/2+.85,b['center']['y'],math.pi]
 root=args.output;runtime=backend=None;report=dict(status='starting',class_name='cigarette',close_commands_sent=0,release_commands_sent=0,carry_destination_verified=False,object_remains_held=True,navigation_route_changed=False,verification_rows=[])
 def stop(*_):raise KeyboardInterrupt()
 signal.signal(signal.SIGTERM,stop)
 with RobotLock():
  root.mkdir(exist_ok=False);started=time.time_ns();policy_snapshot(root)
  with (root/'journal.jsonl').open('x') as log:
   def record(e):log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
   def stage(s):report['stage']=s;write_json(root/'result.json',report)
   try:
    import rclpy
    rclpy.init();epoch='box-carry-'+uuid.uuid4().hex;runtime=TaskCarryRuntime(root,epoch,record);backend=LiveBackend(cfg);backend.bind(root,epoch,record);backend.observation_runtime=runtime
    runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
    holding_feedback(state['joints'],holding)
    report['initial_state']=state;write_json(root/'held-context.json',held);write_json(root/'config.snapshot.json',cfg)
    evidence=HoldEvidence()
    def check(label):
     for index in range(2):
      sample=runtime.sample(processing_reserve_ns=100_000_000);state=runtime.state();holding_feedback(state['joints'],holding)
      proof=evidence.observe(hold_assessment(reference,sample['hand'],appearance),sample['snapshot']['streams']['hand']['stamp_ns'])
      runtime.fresh_measurement(sample['snapshot'])
      if proof['state']!='absent':break
      runtime.pump(.12)
     capture=runtime.save(sample,label)
     report['verification_rows'].append(dict(capture=str(capture),proof=proof));write_json(root/'result.json',report);return sample
    stage('fold_and_tuck_preserving_box_grip')
    goal=runtime.state()['joints'];goal[1]=0.;goal[3]=-math.pi/2;runtime.move_joints(goal,'fold_flat_box_away_from_table')
    goal=runtime.state()['joints'];goal[0]=0.;runtime.move_joints(goal,'lower_tucked_box');check('after-tuck')
    # Nav2 retains its existing .02rad supervision, referenced to the actual
    # accepted hold now, rather than an unnecessarily exact old close value.
    holding=float(runtime.state()['joints'][5]);held['holding_opening_rad']=holding
    write_json(root/'held-context.json',held);runtime.face_source((0.,-.5))
    stage('navigate_existing_route_to_living_room');backend.stage='navigate_destination'
    source_room=backend.room_at(runtime.state()['poses']['map'])
    backend.navigate(source_room,destination['room'],cfg['destination_staging_goal'],holding,'destination-navigation')
    check('destination-held');sample=runtime.sample(profile='rgbd');measured=shape(sample['snapshot'],held);runtime.save(sample,'destination-geometry',measured)
    report.update(status='flat_box_destination_hold_verified',carry_destination_verified=True,final_state=runtime.state(),destination_shape=measured)
   except (Exception,KeyboardInterrupt) as exc:
    report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
    if runtime:
     try:report['last_robot_state']=runtime.state()
     except Exception:pass
   finally:
    if backend:backend.close()
    if runtime:runtime.shutdown();rclpy.shutdown()
    report['elapsed_s']=(time.time_ns()-started)/1e9;write_json(root/'result.json',report);print(json.dumps(report))
if __name__=='__main__':main()

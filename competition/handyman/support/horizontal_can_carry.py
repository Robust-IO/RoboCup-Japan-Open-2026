"""Carry the operator-confirmed horizontal-can grasp on the existing route."""
from competition_paths import is_run_path
import argparse,json,math,os,signal,time,uuid
from pathlib import Path
import cv2
import numpy as np
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import LiveBackend
from horizontal_can_hand_trial import HandCanRuntime
from horizontal_can_transport_geometry import context_from_contact,held_silhouette,held_shape
from task_outcome_policy import pose_matches


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',action='store_true');parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not args.run:print(json.dumps(dict(robot_commands_sent=False,status='held_can_carry_plan_only')));return
    base=Path('/__HANDYMAN_PACKAGE__/runtime');grasp=base/'horizontal-can-hand-grasp-20261101-04'
    if (os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or not is_run_path(args.output.resolve())):raise ValueError('domain71_and_D_output_required')
    confirmation=json.loads((base/'phase5-horizontal-can-held-progress-20261101.json').read_text())
    if not confirmation['operator_confirmed_grasp_success']:raise ValueError('confirmed_grasp_required')
    prior=json.loads((grasp/'result.json').read_text());holding=prior['holding_opening_rad']
    model=json.loads((Path(prior['source_reference'])/'source-geometry.json').read_text())
    appearance=json.loads((Path(prior['source_reference'])/'body-appearance.json').read_text())
    locked=json.loads((grasp/'locked-target.json').read_text())
    preclose=json.loads((grasp/'captures/preclose/snapshot.json').read_text())
    held=context_from_contact(preclose,locked,model,holding)
    reference=cv2.imread(str(grasp/'captures/lift-proof-0/hand.png'))
    cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text())
    cfg.update(navigation_joint_feedback_max_age_s=.85,navigation_feedback_recovery=True)
    root=args.output;runtime=backend=None
    report=dict(status='starting',close_commands_sent=0,release_commands_sent=0,prior_close_commands_sent=1,
        grasp_success_verified_by_operator=True,carry_destination_verified=False,object_remains_held=True,
        navigation_profile=cfg['navigation_motion_profile'],navigation_route_changed=False,verification_rows=[])
    def stopped(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stopped)
    with RobotLock():
        root.mkdir(exist_ok=False);started=time.time_ns()
        with (root/'journal.jsonl').open('x') as stream:
            def record(event):
                stream.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');stream.flush();write_json(root/'result.json',report)
            def stage(name):report['stage']=name;write_json(root/'result.json',report)
            try:
                import rclpy
                rclpy.init();epoch='horizontal-can-carry-'+uuid.uuid4().hex
                runtime=HandCanRuntime(root,epoch,record);backend=LiveBackend(cfg);backend.bind(root,epoch,record)
                runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
                if not pose_matches(state['joints'],prior['last_robot_state']['joints']):raise ValueError('actual_confirmed_held_pose_required')
                report['initial_state']=state;write_json(root/'held-context.json',held);write_json(root/'config.snapshot.json',cfg)
                def held_check(label):
                    sample=runtime.sample(processing_reserve_ns=200_000_000);state=runtime.state()
                    if abs(state['joints'][5]-holding)>.03:raise ValueError('holding_opening_changed')
                    proof=held_silhouette(reference,sample['hand'],appearance)
                    runtime.fresh_measurement(sample['snapshot'])
                    capture=runtime.save(sample,label);row=dict(capture=str(capture),proof=proof,state=state)
                    report['verification_rows'].append(row);write_json(root/'result.json',report)
                    return sample,row
                held_check('current-held-before-fold')
                stage('fold_preserving_grip')
                goal=runtime.state()['joints'];goal[1]=0.;goal[3]=-math.pi/2
                runtime.move_joints(goal,'fold_up_away_from_source_table');held_check('after-fold')
                goal=runtime.state()['joints'];goal[4]=0.
                runtime.move_joints(goal,'restore_standard_wrist_roll_preserving_grip');held_check('after-wrist-roll')
                goal=runtime.state()['joints'];goal[0]=0.
                runtime.move_joints(goal,'lower_folded_can_to_navigation_pose');held_check('after-tuck')
                if not pose_matches(runtime.state()['joints'],[0,0,0,-math.pi/2,0,holding]):raise ValueError('standard_tucked_hold_not_observed')
                runtime.face_source((0.,-.5))
                stage('navigate_existing_route_to_living_room');backend.stage='navigate_destination'
                current=runtime.state()
                if backend.room_at(current['poses']['map'])!='bedroom':raise ValueError('source_room_mismatch')
                backend.navigate('bedroom','living_room',cfg['destination_staging_goal'],holding,'destination-navigation')
                held_check('destination-held')
                sample=runtime.sample(processing_reserve_ns=200_000_000)
                shape=held_shape(sample['snapshot'],held);runtime.save(sample,'destination-geometry',shape)
                report.update(status='horizontal_can_destination_hold_verified',carry_destination_verified=True,
                    final_state=runtime.state(),destination_shape=shape)
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
                if runtime:
                    try:report['last_robot_state']=runtime.state()
                    except Exception:pass
            finally:
                if backend:backend.close()
                if runtime:runtime.shutdown();rclpy.shutdown()
                report['elapsed_s']=(time.time_ns()-started)/1e9
                write_json(root/'result.json',report);print(json.dumps(report))

if __name__=='__main__':main()

"""Explicit held-can retraction, optional original-route destination navigation.

Never grasps again, opens the hand, changes navigation tolerances, or places.
Default is a dry plan; current RGBD/TF and grip are checked at each handoff.
"""
import argparse
import json
import os
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true');parser.add_argument('--navigate-destination',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--context',type=Path)
    parser.add_argument('--owner-lock-fd',type=int);parser.add_argument('--owner-run-root',type=Path)
    args=parser.parse_args()
    if not args.run:
        print(json.dumps(dict(status='dry_carry_plan',robot_commands_sent=False,
            stages=['new_held_rgbd','fold_lower_without_release','new_tucked_held_rgbd']+
                (['original_bedroom_to_living_route','new_destination_held_rgbd'] if args.navigate_destination else []),
            grasp_or_close_repeated=False,grip_commands_sent=False,placement_supported=False)))
        return
    if args.output is None or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('explicit simulation domain71/localhost and NEW D-drive output required')
    import numpy as np
    import rclpy
    from apple_transfer_automation import RobotLock,check_inputs,write_json
    from apple_transfer_auto_backend import LiveBackend
    from canned_grasp_runtime import CanRuntime
    from canned_carry_geometry import current_held,carry_initial,tucked_holding,compare_held,current_images_and_feedback
    from can_transfer_context import stage_lock,load_context
    code=Path(__file__).parent;base=Path('/mnt/d/handyman-training-runs');root=args.output
    root.mkdir(parents=True,exist_ok=False)
    checkpoint=args.context or base/'canned-grasp-lift-live-progress-20261023.json'
    reference=load_context(checkpoint,root) if args.context else json.loads(checkpoint.read_text())
    cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
    if reference['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
        raise ValueError('new_boot_requires_new_held_scene_audit')
    if args.context:contact_start_lift=reference['contact_start_lift_m']
    else:
        source=Path(reference['source_approach'].replace('D:/','/mnt/d/'))
        contact_start_lift=json.loads((source/'result.json').read_text())['grasp_plan']['contact_start_lift_m']
    holding=reference['held_opening_rad'];epoch=root.name+'-rgbd';runtime=None;backend=None
    report=dict(status='starting',class_name='canned_juice',grasp_predecessor=str(checkpoint),
        holding_opening_rad=holding,grasp_or_close_repeated=False,grip_commands_sent=False,
        hand_released=False,automatic_action_retry=False,task_finished_sent=False,placement_verified=False,
        retraction_verified=False,destination_navigation_verified=False,goal_tolerances_changed=False)
    report['context_run_id']=reference.get('run_id');appearance=None
    if args.context:
        import cv2
        from held_can_appearance import build_appearance
        source_capture=Path(reference['source_capture'])
        appearance=build_appearance(cv2.cvtColor(cv2.imread(str(source_capture/'rgb.png')),cv2.COLOR_BGR2RGB),
            np.load(source_capture/'depth_m.npy',allow_pickle=False),json.loads((source_capture/'snapshot.json').read_text()),
            json.loads((source_capture/'can-measurement.json').read_text()))
    def save():write_json(root/'result.json',report)
    with stage_lock(args.owner_lock_fd,args.owner_run_root),(root/'journal.jsonl').open('x') as stream:
        def record(event):
            value=dict(time_ns=time.time_ns(),**event)
            stream.write(json.dumps(value,allow_nan=False)+'\n');stream.flush();print(json.dumps(value),flush=True)
        def stage(name):report['stage']=name;save();record(dict(event='stage',stage=name))
        try:
            write_json(root/'input-audit.json',check_inputs(cfg));rclpy.init()
            runtime=CanRuntime(root,epoch,record)
            def observe(label,after_ns=0,allow_rgb=False):
                rows=[];failures=[];end=time.monotonic()+8
                while len(rows)<3 and time.monotonic()<end:
                    # No detector runs here. Keep the final original400ms check
                    # without charging a200ms model budget to the joint stream.
                    sample=runtime.sample(processing_reserve_ns=0);s=sample['snapshot']
                    if min(s['streams'][k]['stamp_ns'] for k in ('rgb','depth'))<=after_ns:continue
                    try:
                        try:held=current_held(sample['depth'],s,reference,**(dict(rgb=sample['rgb'],appearance=appearance) if appearance else {}))
                        except ValueError:
                            if not allow_rgb or appearance is None:raise
                            from held_can_appearance import assess
                            held=assess(sample['rgb'],sample['depth'],s,reference,appearance)
                            held['opening_rad']=s['joints']['hand_motor_joint']
                        feedback=runtime.state()
                        timing=current_images_and_feedback(s,feedback,runtime.node.get_clock().now().nanoseconds,epoch)
                        held.update(timing=timing,captured_opening_rad=held['opening_rad'],opening_rad=feedback['joints'][5])
                        stamp=s['streams']['depth']['stamp_ns']
                        if rows and stamp-rows[-1]['stamp_ns']<200_000_000:continue
                        path=runtime.save(sample,f'{label}-{len(rows):02d}',held.get('measurement'))
                        rows.append(dict(held,capture=str(path),stamp_ns=stamp))
                    except ValueError as exc:
                        path=runtime.save(sample,f'{label}-rejected-{len(failures):02d}')
                        failures.append(dict(reason=str(exc),capture=str(path)))
                if len(rows)!=3:raise ValueError('new_can_hold_not_observed:'+label+':'+str(failures[-2:]))
                depth_rows=[r for r in rows if r.get('current_depth_surface_verified')]
                if depth_rows:
                    offset=np.asarray([r['center_minus_palm_base_m'] for r in depth_rows])
                    if np.max(np.linalg.norm(offset-offset.mean(0),axis=1))>.02:raise ValueError('can_hold_not_stable:'+label)
                rgb_rows=[r for r in rows if r.get('current_rgb_appearance_verified')]
                if len(rgb_rows)>1:
                    centers=np.asarray([r['cap']['center_pixel'] for r in rgb_rows])
                    if np.max(np.linalg.norm(centers-centers.mean(0),axis=1))>20:raise ValueError('can_rgb_hold_not_stable:'+label)
                value=dict(rows=rows,rejected=failures,current_can_held_verified=True)
                write_json(root/(label+'.json'),value);return rows[-1]
            stage('new_held_rgbd_before_retraction');before=observe('before-retract')
            if args.context:
                import cv2
                from held_can_appearance import build_appearance
                capture=Path(before['capture'])
                appearance=build_appearance(cv2.cvtColor(cv2.imread(str(capture/'rgb.png')),cv2.COLOR_BGR2RGB),
                    np.load(capture/'depth_m.npy',allow_pickle=False),json.loads((capture/'snapshot.json').read_text()),before['measurement'])
                write_json(root/'appearance-source.json',dict(appearance=appearance,source_capture=str(capture)))
            def compare(before,after):
                if after.get('current_depth_surface_verified'):return compare_held(before,after)
                return dict(can_still_held_verified=after.get('current_rgb_appearance_verified') is True,
                    evidence_type=after['evidence_type'],current_object3d_measured=False,placement_verified=False)
            state=runtime.state();carry_initial(state['joints'],contact_start_lift,holding)
            stage('fold_lower_preserving_grip')
            carry=runtime.child([code/'pregrasp_raised_trial.py','--run','--segmented-high',
                '--simulation-contact-trial','--bedroom-side-table-grasp','--motion-profile','balanced_v1',
                '--efficient-transfer','--reviewed-map-pose',*state['poses']['map'],
                '--measured-contact-start-lift',contact_start_lift,'--contact-finish','carry',
                '--holding-opening',holding],'carry-retract')
            completed=time.time_ns()
            if any(e.get('event')=='command' and e.get('kind')=='hand' for e in carry.get('events',[])):
                raise ValueError('unexpected_hand_command_during_retraction')
            stage('new_tucked_held_rgbd');after=observe('after-retract',completed,allow_rgb=bool(args.context))
            state=runtime.state()
            if not tucked_holding(state['joints'],holding):raise ValueError('actual_tucked_holding_pose_not_reached')
            report.update(retraction_verified=True,retraction_evidence=carry,
                retraction_hold_comparison=compare(before,after),post_retraction_state=state)
            save()
            if args.navigate_destination:
                backend=LiveBackend(cfg);backend.bind(root,root.name,record);backend.stage='navigate_destination'
                if backend.room_at(state['poses']['map'])!='bedroom':raise ValueError('can_carry_not_in_source_bedroom')
                stage('original_route_to_destination')
                report['navigation']=backend.navigate('bedroom','living_room',cfg['destination_staging_goal'],holding,'destination-navigation')
                arrived=time.time_ns();stage('new_destination_held_rgbd')
                destination=observe('destination-held',arrived,allow_rgb=bool(args.context))
                state=runtime.state()
                if not tucked_holding(state['joints'],holding):raise ValueError('destination_held_pose_changed')
                report.update(destination_navigation_verified=True,
                    destination_hold_comparison=compare(before,destination),destination_state=state,
                    destination_id=cfg['destination_id'])
            report.update(status='can_retraction_and_destination_navigation_verified' if args.navigate_destination else 'can_retraction_with_grip_verified',
                can_remains_held=True,robot_commands_sent=True,full_transfer_verified=False)
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped_requires_current_state_review',reason=str(exc) or 'interrupted',
                automatic_resume=False)
        finally:
            if backend:backend.close()
            if runtime:
                try:report['last_fresh_robot_state']=runtime.state()
                except Exception as exc:report['state_read_error']=str(exc)
                runtime.shutdown()
            if rclpy.ok():rclpy.shutdown()
            save();print(json.dumps(report,allow_nan=False),flush=True)
    if not report['retraction_verified'] or args.navigate_destination and not report['destination_navigation_verified']:
        raise SystemExit(1)


if __name__=='__main__':main()

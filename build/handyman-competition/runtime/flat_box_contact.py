"""Continue this task's open, near-contact box pose without replaying approach."""
from competition_paths import is_run_path
import argparse,json,math,os,time,uuid
from pathlib import Path
import numpy as np
from flat_box_grasp import BoxRuntime
from flat_box_partial_top import measure,locked_patch
from flat_box_geometry import measure_near
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_geometry import transform_point
from hand_camera_ray import rotation
from thin_box_clearance import ThinBoxClearance,plane_in_map
from task_outcome_policy import pose_matches
from task_outcome_policy import hand_is_open
from outcome_transfer_policy import snapshot as policy_snapshot,alignment_tolerances
from can_destination_surface import measure as measure_table

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--previous',type=Path,required=True);p.add_argument('--gripper-geometry',type=Path,required=True);args=p.parse_args()
    if not args.run:return
    base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
    if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('isolated_domain71_required')
    for path in (args.output,args.previous,args.gripper_geometry):
        if not is_run_path(path.resolve()):raise ValueError('D_paths_required')
    previous=json.loads((args.previous/'result.json').read_text());model=json.loads((args.previous/'source-geometry.json').read_text())
    appearance=json.loads((args.previous/'appearance.json').read_text());source=json.loads((args.previous/'source-snapshot.json').read_text())
    candidates=list((args.previous/'captures').glob('box-*'))+list((args.previous/'captures').glob('contact-*'))
    last=max(candidates,key=lambda p:json.loads((p/'snapshot.json').read_text())['streams']['depth']['stamp_ns'])
    old_snapshot=json.loads((last/'snapshot.json').read_text());old_measurement=json.loads((last/'object-measurement.json').read_text())
    center=transform_point(old_snapshot,old_measurement['center_m'])
    axis_odom=rotation(source['tracking_transforms']['odom_from_base_at_depth']['quaternion'])@model['axis_base']
    plane=plane_in_map(source,model['support_plane_base']);safety=ThinBoxClearance(args.gripper_geometry)
    root=args.output;runtime=None;serial=0;start=time.time_ns()
    report=dict(status='starting',class_name='cigarette',close_commands_sent=0,release_commands_sent=0,approach_commands_sent=0,
        grasp_success_verified=False,continued_from_open_predecessor=str(args.previous),source_model_detection_used=True,
        source_table_verified=previous['source_table_verified'],thin_object_no_table_contact_policy=True,
        model_changed=False,navigation_parameters_changed=False,operator_confirmed_test_object=False)
    with RobotLock():
        root.mkdir(exist_ok=False);policy=policy_snapshot(root)
        with (root/'journal.jsonl').open('x') as log:
            def record(event):
                if event.get('event')=='can_closure_sent':report['close_commands_sent']+=1
                if event.get('event')=='finite_can_approach_sent':report['approach_commands_sent']+=1
                log.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');log.flush();write_json(root/'result.json',report)
            try:
                import rclpy
                rclpy.init();runtime=BoxRuntime(root,'box-contact-'+uuid.uuid4().hex,record)
                runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
                if (previous['close_commands_sent'] or previous['release_commands_sent']
                    or previous.get('class_name')!='cigarette'
                    or not hand_is_open(state['joints'][5])):raise ValueError('unreleased_ungrasped_open_predecessor_required')
                report['initial_state']=state
                cfg=json.loads((runtime.code/'config/apple_transfer_auto_v1.json').read_text())
                runtime.source_table=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==report['source_table_verified'])
                runtime.box_safety=safety;runtime.box_model=model;runtime.box_axis_odom=axis_odom
                runtime.face_source((0.,-.65))
                write_json(root/'source-snapshot.json',source);write_json(root/'source-geometry.json',model);write_json(root/'appearance.json',appearance)
                width=abs(model['axis_base'][1])*model['length_m']+abs(model['axis_base'][0])*model['width_m']
                runtime.open_side_margin_m=(.093026-width)/2;runtime.lateral_tolerance_m=alignment_tolerances(runtime.open_side_margin_m)[1]
                def observe():
                    nonlocal serial,center
                    reason=None
                    for _ in range(3):
                        sample=runtime.sample(processing_reserve_ns=100_000_000);s=sample['snapshot']
                        expected=transform_point(s,center,inverse=True)
                        axis=rotation(s['tracking_transforms']['odom_from_base_at_depth']['quaternion']).T@axis_odom
                        try:
                            try:m=measure_near(sample['rgb'],sample['depth'],s,expected,model,appearance)
                            except ValueError as exc:
                                if str(exc)!='flat_box_top_too_occluded_to_center':raise
                                try:m=measure(sample['rgb'],sample['depth'],s,expected,axis,model,appearance)
                                except ValueError:m=locked_patch(sample['rgb'],sample['depth'],s,expected,axis,model,appearance)
                            current_plane=measure_table(sample['depth'],s,runtime.source_table)['plane_map']
                            proof=safety.check(s,runtime.palm_pose(s),m['center_m'],model['height_m'],current_plane)
                            if proof['actual_palm_height_map_m']>proof['maximum_palm_height_for_contact_m']:raise ValueError('actual_fingers_above_thin_box_contact_band')
                            runtime.accept_box(m,s);runtime.fresh_measurement(s)
                            center=transform_point(s,m['center_m']);serial+=1
                            capture=runtime.save(sample,f'contact-{serial:03d}',m)
                            record(dict(event='visible_box_depth_and_table_clearance',measurement=m,clearance=proof,capture=str(capture)))
                            return sample,m
                        except ValueError as exc:reason=str(exc)
                    raise ValueError(reason)
                report['stage']='near_contact_verification'
                arm=state['joints'][:5];sample,m=observe()
                if not runtime.is_aligned(m['center_m'],sample['snapshot']):
                    if m.get('position_xy_is_prediction'):raise ValueError('locked_contact_pose_not_aligned_no_blind_approach')
                    if previous['approach_commands_sent']>=policy['maximum_corrections']+1:raise ValueError('cumulative_initial_plus_five_exhausted')
                    runtime.approach(observe,arm,previous['approach_commands_sent'])
                report['stage']='close_once';write_json(root/'result.json',report)
                closure,_,_=runtime.close_gripper(observe,arm);holding=float(closure['closure_evidence']['observed_opening_rad'])
                baseline,m0=observe();s0=baseline['snapshot'];c0=transform_point(s0,m0['center_m']);p0=transform_point(s0,s0['palm_base_at_depth'])
                v0=transform_point(s0,m0.get('visible_surface_centroid_m',m0['center_m']))
                report['holding_opening_rad']=holding;report['stage']='lift_and_verify_actual_box_depth'
                goal=runtime.state()['joints'];goal[0]+=.08;goal[5]=holding;runtime.move_joints(goal,'lift_thin_box_with_table_clearance')
                rows=[]
                for index in range(policy['verification']['lift_frames']):
                    if index:runtime.pump(policy['verification']['frame_separation_s'])
                    sample=runtime.sample(processing_reserve_ns=100_000_000);s=sample['snapshot'];p1=transform_point(s,s['palm_base_at_depth'])
                    expected=transform_point(s,c0+p1-p0,inverse=True)
                    axis=rotation(s['tracking_transforms']['odom_from_base_at_depth']['quaternion']).T@axis_odom
                    try:m=measure_near(sample['rgb'],sample['depth'],s,expected,model,appearance)
                    except ValueError as exc:
                        if str(exc)!='flat_box_top_too_occluded_to_center':raise
                        try:m=measure(sample['rgb'],sample['depth'],s,expected,axis,model,appearance)
                        except ValueError:m=locked_patch(sample['rgb'],sample['depth'],s,expected,axis,model,appearance)
                    c1=transform_point(s,m.get('visible_surface_centroid_m',m['center_m']));lift=float((c1-v0)[2]);error=float(np.linalg.norm((c1-v0)-(p1-p0)))
                    if lift<policy['verification']['minimum_lift_m'] or error>policy['verification']['follow_error_m']:raise ValueError('actual_box_did_not_follow_lift')
                    capture=runtime.save(sample,f'lift-proof-{index}',m);rows.append(dict(capture=str(capture),measurement=m,observed_lift_m=lift,follow_error_m=error))
                palm=runtime.palm_pose(s)
                held=dict(class_name='cigarette',grasp_lift_verified=True,holding_opening_rad=holding,
                    center_palm_m=(rotation(palm['quaternion']).T@(np.asarray(m['center_m'])-palm['translation'])).tolist(),
                    model=model,appearance=appearance,source_run=str(root),source='actual_visible_RGBD_surface_lift_observations_with_locked_XY_when_occluded')
                write_json(root/'held-context.json',held)
                report.update(status='flat_box_grasp_and_lift_verified',grasp_success_verified=True,lift_evidence=rows,object_remains_held=True,final_state=runtime.state())
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped_requires_actual_state_review',reason=str(exc))
                if runtime:
                    try:report['last_robot_state']=runtime.state()
                    except Exception:pass
            finally:
                if runtime:runtime.shutdown();rclpy.shutdown()
                report['elapsed_s']=(time.time_ns()-start)/1e9;write_json(root/'result.json',report);print(json.dumps(report))
if __name__=='__main__':main()

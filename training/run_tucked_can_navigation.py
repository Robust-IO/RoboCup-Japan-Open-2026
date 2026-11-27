"""Held-can observation + ORIGINAL navigation ONLY. No arm/gripper commands.

Never replays grasp/retraction. The RGB exception only continues a verified
held object when its cap is depth-clipped; no3D point is inferred from RGB.
"""
import argparse
import json
import os
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--observe-only',action='store_true')
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if args.run and args.observe_only:p.error('choose run or readonly observation')
    if not args.run and not args.observe_only:
        print(json.dumps(dict(status='dry_tucked_can_navigation_plan',robot_commands_sent=False,
            arm_or_gripper_commands_supported=False,uses_original_route=True,placement_supported=False,
            held_rgb_exception_does_not_create_new_object_identity=True)))
        return
    if args.output is None or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('explicit reviewed domain71/localhost and new D-drive output required')
    import hashlib
    import cv2
    import numpy as np
    import rclpy
    from apple_transfer_automation import RobotLock,check_inputs,write_json
    from apple_transfer_auto_backend import LiveBackend
    from canned_grasp_runtime import CanRuntime
    from canned_carry_geometry import current_held,current_images_and_feedback,tucked_holding
    from held_can_appearance import build_appearance,assess
    code=Path(__file__).parent;base=Path('/mnt/d/handyman-training-runs');root=args.output
    root.mkdir(parents=True,exist_ok=False)
    previous=base/'canned-carry-retracted-progress-20261023.json'
    checkpoint=json.loads(previous.read_text());reference=json.loads((base/'canned-grasp-lift-live-progress-20261023.json').read_text())
    if (not checkpoint.get('physical_arm_retraction_completed') or not reference.get('can_grasp_and_lift_verified')
            or checkpoint['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()):
        raise ValueError('verified_same_boot_retracted_can_context_required')
    carry_root=Path(checkpoint['actual_carry_trial'].replace('D:/','/mnt/d/'))
    before=json.loads((carry_root/'before-retract.json').read_text())
    if not before.get('current_can_held_verified') or len(before.get('rows',[]))<3:
        raise ValueError('actual_held_rgbd_template_required')
    source=Path(before['rows'][-1]['capture'])
    names=('rgb.png','depth_m.npy','snapshot.json','can-measurement.json')
    source_hashes={name:hashlib.sha256((source/name).read_bytes()).hexdigest() for name in names}
    source_rgb=cv2.cvtColor(cv2.imread(str(source/'rgb.png')),cv2.COLOR_BGR2RGB)
    appearance=build_appearance(source_rgb,np.load(source/'depth_m.npy',allow_pickle=False),
        json.loads((source/'snapshot.json').read_text()),json.loads((source/'can-measurement.json').read_text()))
    write_json(root/'appearance-source.json',dict(appearance=appearance,source_capture=str(source),source_sha256=source_hashes))
    cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
    report=dict(status='starting',mode='navigation_only' if args.run else 'observation_only',
        arm_commands_sent=False,gripper_commands_sent=False,grasp_or_retraction_replayed=False,
        placement_verified=False,hand_released=False,task_finished_sent=False,automatic_action_retry=False,
        navigation_controller_verified=False,destination_held_verified=False)
    runtime=None;backend=None
    def save():write_json(root/'result.json',report)
    with RobotLock(),(root/'journal.jsonl').open('x') as stream:
        def record(event):
            value=dict(time_ns=time.time_ns(),**event);stream.write(json.dumps(value,allow_nan=False)+'\n');stream.flush()
            if value['event']!='frame_rejected':print(json.dumps(value),flush=True)
        def stage(name):report['stage']=name;save();record(dict(event='stage',stage=name))
        try:
            write_json(root/'input-audit.json',check_inputs(cfg));rclpy.init()
            runtime=CanRuntime(root,root.name,record)
            def observe(label,after_ns=0):
                rows=[];rejected=[];end=time.monotonic()+8
                while len(rows)<3 and time.monotonic()<end:
                    sample=runtime.sample(processing_reserve_ns=0);snapshot=sample['snapshot']
                    stamp=snapshot['streams']['depth']['stamp_ns']
                    if min(snapshot['streams'][key]['stamp_ns'] for key in ('rgb','depth'))<=after_ns:continue
                    try:
                        try:
                            evidence=current_held(sample['depth'],snapshot,reference)
                            evidence['evidence_type']='actual_current_rgbd_surface'
                        except ValueError:
                            evidence=assess(sample['rgb'],sample['depth'],snapshot,reference,appearance)
                        feedback=runtime.state()
                        timing=current_images_and_feedback(snapshot,feedback,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
                        if not tucked_holding(feedback['joints'],reference['held_opening_rad']):raise ValueError('current_hand_not_tucked_holding')
                        if rows and stamp-rows[-1]['stamp_ns']<200_000_000:continue
                        path=runtime.save(sample,f'{label}-{len(rows):02d}')
                        item=dict(stamp_ns=stamp,capture=str(path),evidence=evidence,timing=timing,current_feedback=feedback)
                        write_json(path/'held-evidence.json',item);rows.append(item)
                    except ValueError as exc:
                        path=runtime.save(sample,f'{label}-rejected-{len(rejected):02d}')
                        rejected.append(dict(reason=str(exc),capture=str(path)))
                        record(dict(event='frame_rejected',reason=str(exc),capture=str(path)))
                if len(rows)!=3:raise ValueError('current_held_can_not_verified:'+label+':'+str(rejected[-2:]))
                rgb_rows=[row['evidence'] for row in rows if row['evidence'].get('current_rgb_appearance_verified')]
                if len(rgb_rows)==3:
                    centers=np.asarray([e['cap']['center_pixel'] for e in rgb_rows])
                    if np.max(np.linalg.norm(centers-centers.mean(0),axis=1))>20:raise ValueError('current_cap_not_temporally_stable')
                result=dict(status='current_held_can_observed',rows=rows,rejected=rejected,
                    rgb_only_does_not_measure_object3d=True,robot_commands_sent=False)
                write_json(root/(label+'.json'),result);return result
            stage('verify_current_tucked_can');report['before_navigation_hold']=observe('before-navigation')
            if args.observe_only:
                report.update(status='current_tucked_can_appearance_verified',robot_commands_sent=False)
            else:
                state=runtime.state();backend=LiveBackend(cfg);backend.bind(root,root.name,record);backend.stage='navigate_destination'
                if backend.room_at(state['poses']['map'])!='bedroom':raise ValueError('current_robot_not_in_source_bedroom')
                stage('original_route_to_square_table')
                report['navigation']=backend.navigate('bedroom','living_room',cfg['destination_staging_goal'],
                    reference['held_opening_rad'],'destination-navigation')
                report['navigation_controller_verified']=True;arrived=time.time_ns();save()
                stage('verify_destination_tucked_can');report['after_navigation_hold']=observe('after-navigation',arrived)
                report.update(status='original_route_can_navigation_and_current_hold_verified',
                    destination_held_verified=True,robot_commands_sent=True,full_transfer_verified=False)
            if any(hashlib.sha256((source/name).read_bytes()).hexdigest()!=digest for name,digest in source_hashes.items()):
                raise ValueError('verified_template_recording_changed')
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped_requires_current_state_review',reason=str(exc) or 'interrupted',automatic_resume=False)
        finally:
            if backend:backend.close()
            if runtime:
                try:report['last_fresh_robot_state']=runtime.state()
                except Exception as exc:report['state_read_error']=str(exc)
                runtime.shutdown()
            if rclpy.ok():rclpy.shutdown()
            save();print(json.dumps({k:v for k,v in report.items() if k not in ('before_navigation_hold','after_navigation_hold')}),flush=True)
    if report['status'] not in ('current_tucked_can_appearance_verified','original_route_can_navigation_and_current_hold_verified'):raise SystemExit(1)


if __name__=='__main__':main()

"""Own one live observation process and one non-replayed close/lift sequence.

Explicit Unity/domain71 experiment. A contact candidate is only permission for
the verification lift, never proof that the object was successfully grasped.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from apple_approach_lease import read_latest
from hand_camera_ray import rotation
from latest_capture_index import latest
from motion_speed import load_profile


def lift_allowed(result):
    evidence=result.get('closure_evidence',{})
    opening=evidence.get('observed_opening_rad',float('nan'))
    return (result.get('status')=='passed' and result.get('grasp_closure_commanded') is True
        and evidence.get('state')=='contact_candidate' and math.isfinite(opening) and .02<opening<=.62
        and evidence.get('stable_samples',0)>=4 and evidence.get('stable_duration_s',0)>=.6
        and evidence.get('closing_travel_rad',0)>=.03)


def owned_observer_command(code,recording,epoch,args):
    command=[sys.executable,str(code/'capture_grasp_scene.py'),str(recording),'--hand-geometry','--tracking-epoch',epoch,
        '--duration','90','--interval','.1','--inference-python',args.model_python,'--registry',args.registry,'--inference-imgsz','960',
        '--environment-yaml',args.environment,'--expected-layout','LayoutA','--target-room',args.room,
        '--identity-tracking-task',args.tracking_task_id,'--compact-progress']
    if args.confirm_apple_roi:command+=['--confirm-apple-roi']
    if getattr(args,'near_apple_geometry',False):
        command+=['--near-apple-geometry','--near-apple-gap-m',str(args.near_apple_gap_m)]
    return command


def add_independent_lift_verification(directory,recording,epoch,task,*,external_observer):
    """New explicit evidence; never rewrites the original detector-only verdict."""
    from audit_preclose_identity_lift_follow import audit
    path=Path(directory)/'independent-lift-verification.json'
    if path.exists():raise ValueError('independent_verification_already_exists')
    verification=audit(directory,expected_recording=recording if external_observer else None,
                       expected_epoch=epoch,expected_task_id=task)
    with path.open('x') as stream:json.dump(verification,stream,indent=2,allow_nan=False)
    return dict(effective_grasp_success_verified=verification['grasp_success_verified'],
                effective_verification_evidence=str(path),
                effective_verification_method='confirmed_preclose_identity_and_measured_rgbd_lift_follow',
                canonical_detector_verdict_preserved=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model-python',required=True);p.add_argument('--registry',required=True);p.add_argument('--environment',required=True)
    p.add_argument('--map-pose',type=float,nargs=3,required=True)
    p.add_argument('--room',choices=['living_room','bedroom'],default='living_room')
    p.add_argument('--contact-start-lift',type=float,default=.25268720033)
    p.add_argument('--observer-recording',type=Path);p.add_argument('--observer-epoch');p.add_argument('--tracking-task-id',default='apple-white-table-20261017')
    p.add_argument('--confirm-apple-roi',action='store_true',help='Opt in for a NEW owned near-grasp ROI observer; never restore an old identity')
    p.add_argument('--near-apple-geometry',action='store_true',help='Opt in for a NEW owned near-grasp current-RGBD handoff')
    p.add_argument('--near-apple-gap-m',type=float,default=.15)
    p.add_argument('--verify-preclose-rgbd-follow',action='store_true',help='Explicit independent post-lift RGBD evidence bound to this current task/recording; preserve detector-only verdict')
    p.add_argument('--motion-profile',choices=['baseline','balanced_v1'],default='baseline')
    p.add_argument('--efficient-transfer',action='store_true');a=p.parse_args()
    load_profile(a.motion_profile)
    if bool(a.observer_recording)!=bool(a.observer_epoch):raise ValueError('existing_observer_requires_recording_and_epoch')
    if a.confirm_apple_roi and a.observer_recording:raise ValueError('roi_flag_requires_new_owned_observer')
    if a.near_apple_geometry:
        from near_apple_geometry import check_gap
        check_gap(a.near_apple_gap_m)
        if a.observer_recording:raise ValueError('geometry_flag_requires_new_owned_observer')
    if not math.isfinite(a.contact_start_lift) or not .24<=a.contact_start_lift<=.30:raise ValueError('unreviewed_contact_height')
    if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71':raise ValueError('Explicit live simulation invocation required')
    a.output.mkdir(parents=True,exist_ok=False)
    code=Path(__file__).parent;recording=a.observer_recording or a.output/'observe';epoch=a.observer_epoch or a.output.name;task=a.tracking_task_id
    report=dict(grasp_success_verified=False,automatic_retry=False,commands=[],observer_owned=not bool(a.observer_recording),observation_directory=str(recording),owned_near_grasp_roi_enabled=a.confirm_apple_roi)
    report.update(tracking_epoch=epoch,tracking_task_id=task,preclose_rgbd_follow_enabled=a.verify_preclose_rgbd_follow)
    log=(a.output/'observer.log').open('x');observer=None
    try:
        command=owned_observer_command(code,recording,epoch,a)
        if not a.observer_recording:observer=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+105
        while True:
            if observer is not None and observer.poll() is not None:raise RuntimeError('observer_exited_before_closure')
            try:paths=[latest(recording)]
            except FileNotFoundError:paths=[]
            if paths:
                try:
                    snapshot=json.loads((paths[-1].parent/'snapshot.json').read_text())
                    tf=snapshot['tracking_transforms']['odom_from_base_at_depth'];r=rotation(tf['quaternion'])
                    read_latest(recording,closure=True,now_ns=time.time_ns(),epoch=epoch,task_id=task,
                        current_odom=[*tf['translation'][:2],math.atan2(r[1,0],r[0,0])],palm_height=.2771978151685961+a.contact_start_lift,expected_room=a.room)
                    break
                except (ValueError,KeyError,FileNotFoundError):pass
            if time.monotonic()>deadline:raise RuntimeError('no_fresh_aligned_identity_before_close')
            time.sleep(.05)
        common=[sys.executable,str(code/'pregrasp_raised_trial.py'),'--run','--segmented-high','--simulation-contact-trial',
            '--measured-contact-start-lift',str(a.contact_start_lift),'--reviewed-map-pose',*map(str,a.map_pose),
            '--motion-profile',a.motion_profile]
        if a.room=='bedroom':common+=['--bedroom-side-table-grasp']
        if a.efficient_transfer:
            if a.room!='bedroom':raise ValueError('efficient_close_lift_requires_measured_bedroom_transfer')
            common.append('--efficient-transfer')
        close_dir=a.output/'close'
        print('Live identity ready; executing one closure',flush=True)
        subprocess.run(common+['--contact-finish','close','--identity-recording',str(recording),
            '--tracking-epoch',epoch,'--tracking-task-id',task,'--output',str(close_dir)],check=False,timeout=40)
        close=json.loads((close_dir/'result.json').read_text());report['close']=close
        report['commands']=[e for e in close.get('events',[]) if e.get('event')=='command']
        if not lift_allowed(close):raise RuntimeError('closure_not_verified_no_lift')
        opening=close['closure_evidence']['observed_opening_rad']
        print('Stable contact candidate; lifting10cm for visual verification',flush=True)
        lift_dir=a.output/'lift'
        subprocess.run(common+['--contact-finish','lift','--holding-opening',str(opening),
            '--output',str(lift_dir)],check=False,timeout=40)
        lift=json.loads((lift_dir/'result.json').read_text());report['lift']=lift
        report['commands'] += [e for e in lift.get('events',[]) if e.get('event')=='command']
        if lift.get('status')!='passed':raise RuntimeError('lift_endpoint_not_verified')
        report['lift_completed_at_ns']=time.time_ns()
        # Keep observer alive after the endpoint, to collect settled object evidence.
        deadline=time.monotonic()+4
        post_lift_stamps=[]
        while time.monotonic()<deadline:
            if observer is not None and observer.poll() is not None:break
            if a.efficient_transfer:
                from transfer_efficiency import burst_complete
                from lift_settled_window import contains
                try:
                    capture=latest(recording).parent
                    snapshot=json.loads((capture/'snapshot.json').read_text())
                except FileNotFoundError:
                    snapshot=None
                if snapshot is not None and contains(snapshot,report['lift_completed_at_ns'],time.time_ns()):
                    stamp=snapshot['streams']['depth']['stamp_ns']
                    if not post_lift_stamps or stamp>post_lift_stamps[-1]:post_lift_stamps.append(stamp)
                    if burst_complete(post_lift_stamps,3,.6):break
            time.sleep(.05)
        report['verification_window_end_ns']=time.time_ns()
        report['verification_last_complete_capture']=str(latest(recording).parent.resolve())
        report['status']='closure_contact_and_lift_endpoint_verified_needs_visual_comparison'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',reason=str(exc),requires_actual_pose_audit=True)
    finally:
        if observer is not None:
            if observer.poll() is None:
                os.killpg(observer.pid,signal.SIGINT)
                try:observer.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(observer.pid,signal.SIGTERM)
                    try:observer.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(observer.pid,signal.SIGKILL);observer.wait()
            report['observer_returncode']=observer.returncode
        log.close()
        (a.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    if report['status']=='closure_contact_and_lift_endpoint_verified_needs_visual_comparison':
        try:
            from verify_apple_lift_sequence import verify
            verification=verify(a.output)
            with (a.output/'visual-lift-verification.json').open('x') as stream:
                json.dump(verification,stream,indent=2,allow_nan=False)
            report['grasp_success_verified']=verification['grasp_success_verified']
            report['status']='grasp_lift_verified' if verification['grasp_success_verified'] else 'visual_verification_inconclusive'
            report['verification_evidence']=str(a.output/'visual-lift-verification.json')
        except (ValueError,KeyError,OSError) as exc:
            report.update(status='visual_verification_inconclusive',verification_error=str(exc))
        (a.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    if a.verify_preclose_rgbd_follow and report['status']!='stopped':
        try:
            report.update(add_independent_lift_verification(a.output,recording,epoch,task,
                                                           external_observer=bool(a.observer_recording)))
        except (ValueError,KeyError,OSError) as exc:
            report.update(effective_grasp_success_verified=False,independent_verification_error=str(exc))
        (a.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps(report),flush=True)
    if report['status']=='stopped':raise SystemExit(1)


if __name__=='__main__':main()

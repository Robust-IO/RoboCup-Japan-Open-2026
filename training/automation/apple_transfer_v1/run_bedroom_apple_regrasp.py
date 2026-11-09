"""Own one observer, coarse approach + measured correction, then verified close/lift.

Uses existing geometry, identity, contact and lift checks without lowering gates.
Never navigates, retracts, retries a failed segment or reports Task_finished.
"""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
from apple_approach_lease import measured_target,closure_position_error
from hand_camera_ray import rotation
from latest_capture_index import latest
from motion_speed import load_profile


def forward_approach_delta(target,palm_height,*,single=False):
    """Prefer the actual close region over a millimetre-perfect staging point.

    None means proceed to the existing fresh closure/identity/refit checks,
    NOT permission to close. Outside that region, legacy forward-only limits stay.
    """
    target=np.asarray(target,float)
    if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('invalid_target_position')
    if not math.isfinite(palm_height) or abs(target[2]-palm_height)>.025:raise RuntimeError('measured_height_changed')
    try:closure_position_error(target,palm_height)
    except ValueError as exc:
        if str(exc)!='target_outside_reviewed_closure_region':raise
    else:return None
    delta=target[:2]-[.704,.078];length=float(np.linalg.norm(delta))
    if length<.004:return None
    if delta[0]<=0:raise RuntimeError('requires_reviewed_reposition_not_forward_approach')
    if not single and length>.24:delta*=.22/length
    return delta


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True);p.add_argument('--model-python',required=True);p.add_argument('--registry',required=True);p.add_argument('--environment',required=True)
    p.add_argument('--observer-recording',type=Path);p.add_argument('--observer-epoch');p.add_argument('--task-id',default='apple-white-table-20261017')
    p.add_argument('--contact-start-lift',type=float,default=.25268720033)
    p.add_argument('--verify-preclose-rgbd-follow',action='store_true')
    p.add_argument('--motion-profile',choices=['baseline','balanced_v1'],default='baseline')
    p.add_argument('--efficient-transfer',action='store_true');a=p.parse_args()
    load_profile(a.motion_profile)
    if bool(a.observer_recording)!=bool(a.observer_epoch) or not math.isfinite(a.contact_start_lift) or not .24<=a.contact_start_lift<=.30:raise ValueError('invalid_observer_or_measured_grasp_height')
    if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71':raise ValueError('explicit_simulation_required')
    a.output.mkdir(exist_ok=False);code=Path(__file__).parent;recording=a.observer_recording or a.output/'observe';epoch=a.observer_epoch or a.output.name;task=a.task_id;palm_height=.2771978151685961+a.contact_start_lift
    report=dict(status='starting',grasp_success_verified=False,automatic_retry=False,approach=[]);observer=None;log=(a.output/'observer.log').open('x')
    def save():
        (a.output/'result.json').write_text(json.dumps(report,indent=2))
    def fresh(timeout,closure=False,after_ns=None):
        end=time.monotonic()+timeout;reason='no_observation'
        while time.monotonic()<end:
            if observer is not None and observer.poll() is not None:raise RuntimeError('observer_ended:'+reason)
            try:paths=[latest(recording)]
            except FileNotFoundError:paths=[]
            if paths:
                try:
                    path=paths[-1];s=json.loads((path.parent/'snapshot.json').read_text());d=json.loads(path.read_text());tf=s['tracking_transforms']['odom_from_base_at_depth'];r=rotation(tf['quaternion']);odom=[*tf['translation'][:2],math.atan2(r[1,0],r[0,0])]
                    if after_ns is not None and min(s['streams'][key]['stamp_ns'] for key in ('rgb','depth'))<=after_ns:
                        raise ValueError('post_approach_observation_required')
                    evidence=measured_target(d,s,now_ns=time.time_ns(),epoch=epoch,task_id=task,current_odom=odom,expected_room='bedroom')
                    if closure:
                        from apple_approach_lease import validate_closure
                        validate_closure(d,s,now_ns=time.time_ns(),epoch=epoch,task_id=task,current_odom=odom,palm_height=palm_height,expected_room='bedroom')
                    return evidence,s
                except (ValueError,KeyError,FileNotFoundError) as exc:reason=str(exc)
            time.sleep(.02)
        raise RuntimeError('fresh_geometry_wait_expired:'+reason)
    def approach(delta,s,previous=None):
        mt=s['tracking_transforms']['map_from_base_at_depth'];mr=rotation(mt['quaternion']);pose=[*mt['translation'][:2],math.atan2(mr[1,0],mr[0,0])]
        output=a.output/f'approach-{len(report["approach"]):02d}.json'
        cmd=[sys.executable,str(code/'reviewed_carry_adjustment.py'),'--run','--simulation-ignore-collision','--reviewed-map-pose',*map(str,pose),'--delta',str(delta[0]),str(delta[1]),'0','--expected-arm',str(a.contact_start_lift),'-1.7','0','.1292036732051034','0','--holding-opening','.65','--identity-recording',str(recording),'--tracking-epoch',epoch,'--tracking-task-id',task,'--expected-room','bedroom','--output',str(output),'--motion-profile',a.motion_profile]
        if a.efficient_transfer:
            cmd.append('--efficient-transfer')
            if previous is None:cmd.append('--single-grasp-approach')
            else:cmd+=['--grasp-terminal-correction','--correction-after',previous['evidence_path']]
        print(json.dumps(dict(event='terminal_correction_planned' if previous else 'approach_planned',
                              delta=list(delta),after=previous['evidence_path'] if previous else None)),flush=True)
        if a.efficient_transfer:
            subprocess.run(cmd,check=False)  # Child window derives from actual distance/speed.
        else:
            subprocess.run(cmd,timeout=35,check=False)
        result=json.loads(output.read_text());report['approach'].append(result);save()
        if result.get('status')!='endpoint_verified':raise RuntimeError('approach_endpoint_not_verified')
        return dict(result,evidence_path=str(output))
    try:
        cmd=[sys.executable,str(code/'capture_grasp_scene.py'),str(recording),'--hand-geometry','--tracking-epoch',epoch,'--duration','90','--interval','.1','--inference-python',a.model_python,'--registry',a.registry,'--inference-imgsz','960','--environment-yaml',a.environment,'--expected-layout','LayoutA','--target-room','bedroom','--identity-tracking-task',task,'--compact-progress']
        if not a.observer_recording:observer=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        if a.efficient_transfer:
            evidence,s=fresh(105);target=np.array(evidence['target_base_m'])
            delta=forward_approach_delta(target,palm_height,single=True)
            if delta is not None:
                from grasp_terminal_correction import correct_terminal
                previous=approach(delta,s)
                def observe_after(completed_ns):
                    current,snapshot=fresh(5,after_ns=completed_ns)
                    return current['target_base_m'],snapshot
                report['terminal_alignment']=correct_terminal(observe_after,approach,palm_height,previous)
                save()
        else:
            for index in range(3):
                evidence,s=fresh(105 if index==0 else 5);target=np.array(evidence['target_base_m'])
                delta=forward_approach_delta(target,palm_height)
                if delta is None:break
                approach(delta,s)
        evidence,s=fresh(5,closure=True);mt=s['tracking_transforms']['map_from_base_at_depth'];mr=rotation(mt['quaternion']);pose=[*mt['translation'][:2],math.atan2(mr[1,0],mr[0,0])]
        cmd=[sys.executable,str(code/'run_apple_close_lift_trial.py'),'--run','--model-python',a.model_python,'--registry',a.registry,'--environment',a.environment,'--map-pose',*map(str,pose),'--room','bedroom','--observer-recording',str(recording),'--observer-epoch',epoch,'--tracking-task-id',task,'--contact-start-lift',str(a.contact_start_lift),'--output',str(a.output/'close-lift')]
        cmd+=['--motion-profile',a.motion_profile]
        if a.efficient_transfer:cmd.append('--efficient-transfer')
        if a.verify_preclose_rgbd_follow:cmd+=['--verify-preclose-rgbd-follow']
        # Close/lift owns its observation wait and the bounded close/lift
        # actions; do not cut their combined sequence off at an unrelated65s.
        subprocess.run(cmd,check=False);result=json.loads((a.output/'close-lift/result.json').read_text());report['close_lift']=result
        report['grasp_success_verified']=result.get('effective_grasp_success_verified' if a.verify_preclose_rgbd_follow else 'grasp_success_verified') is True
        report['verification_policy']='bound_preclose_rgbd_follow' if a.verify_preclose_rgbd_follow else 'legacy_detector_only'
        report['status']='grasp_lift_verified' if report['grasp_success_verified'] else 'requires_visual_review'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',reason=str(exc),requires_actual_pose_audit=True)
    finally:
        if observer is not None and observer.poll() is None:
            os.killpg(observer.pid,signal.SIGINT)
            try:observer.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(observer.pid,signal.SIGTERM);observer.wait(timeout=5)
        log.close();save();print(json.dumps(report),flush=True)
    if not report['grasp_success_verified']:raise SystemExit(1)


if __name__=='__main__':main()

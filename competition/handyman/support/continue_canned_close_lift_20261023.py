"""NEW explicit close/lift experiment from audited open, aligned current pose.

Never replays preparation/approach, never repeats a prior close, never releases.
The old reference selects a ROI only; every center is remeasured in NEW depth.
"""
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

CODE=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,check_inputs,write_json
from canned_grasp_runtime import CanRuntime
from canned_grasp_geometry import aligned,fresh,lift_follow,measure_near,predicted_before_contact,transform_point
from task_outcome_policy import pose_matches
import scipy.sparse.csgraph
import rclpy

BASE=Path('/__HANDYMAN_PACKAGE__/runtime');PREV=BASE/'canned-grasp-v1-live-20261023-01'
ROOT=BASE/'canned-close-lift-continuation-20261023-01';ROOT.mkdir(exist_ok=False)
prior=json.loads((PREV/'result.json').read_text());reference=json.loads((PREV/'locked-can-reference.json').read_text())
old_journal=[json.loads(line) for line in (PREV/'journal.jsonl').read_text().splitlines()]
if (prior.get('stage')!='one_closure' or prior.get('close') or (PREV/'closure-contact.json').exists()
        or any(e.get('event') in ('can_closure_intent','can_closure_sent') for e in old_journal)):
    raise ValueError('prior_close_or_ambiguous_publication_must_not_be_repeated')
if Path('/proc/sys/kernel/random/boot_id').read_text().strip()!='03e2ce8b-528d-4517-a5f5-5aa905f81a77':
    raise ValueError('new_boot_requires_new_scene_audit')
report=dict(status='starting_new_close_lift',predecessor=str(PREV),grasp_success_verified=False,
    preparation_replayed=False,approach_replayed=False,close_commands_maximum=1,
    task_finished_sent=False,placement_verified=False,automatic_action_retry=False)
def save():write_json(ROOT/'result.json',report)
with RobotLock(),(ROOT/'journal.jsonl').open('x') as stream:
    def record(event):
        value=dict(time_ns=time.time_ns(),**event);stream.write(json.dumps(value)+'\n');stream.flush();print(json.dumps(value),flush=True)
    rclpy.init();runtime=CanRuntime(ROOT,reference['epoch'],record);serial=[0]
    def observe():
        reason='no_current_can'
        for _ in range(5):
            sample=runtime.sample();snap=sample['snapshot']
            expected=predicted_before_contact(reference,snap,reference['task_id'])
            try:
                value=measure_near(sample['depth'],snap,expected,reference['radius_m'],reference['height_m'])
                fresh(snap,runtime.node.get_clock().now().nanoseconds,reference['epoch'])
                return sample,value
            except ValueError as exc:
                reason=str(exc);path=runtime.save(sample,f'rejected-near-{serial[0]:02d}');serial[0]+=1
                record(dict(event='new_depth_reacquisition_only',reason=reason,capture=str(path)))
        raise ValueError('five_new_depth_frames_rejected:'+reason)
    try:
        cfg=json.loads((CODE/'config/apple_transfer_auto_v1.json').read_text())
        audit=check_inputs(cfg)
        if audit['model_sha256']!=json.loads((PREV/'input-audit.json').read_text())['model_sha256']:
            raise ValueError('identity_model_context_changed')
        write_json(ROOT/'input-audit.json',audit)
        sample,measured=observe();current=runtime.state();previous=prior['last_fresh_robot_state']
        if (math.dist(current['poses']['odom'][:2],previous['poses']['odom'][:2])>.02
                or abs(current['poses']['odom'][2]-previous['poses']['odom'][2])>.03):
            raise ValueError('robot_moved_since_audited_unclosed_stop')
        lift=reference['contact_start_lift_m'];expected_arm=[lift,-1.7,0.,.1292036732051034,0.]
        if not pose_matches(current['joints'],expected_arm+[.65]):raise ValueError('current_open_horizontal_pose_required')
        if not aligned(measured['center_m'],sample['snapshot']):raise ValueError('new_current_can_not_aligned')
        report['stage']='one_new_close';save()
        close,before_sample,before_geometry=runtime.close_gripper(observe,expected_arm)
        report['close']=close;save();opening=close['closure_evidence']['observed_opening_rad']
        if not .02<opening<=.62:raise ValueError('no_current_contact_no_lift')
        report['stage']='verification_lift';save();state=runtime.state()
        report['lift']=runtime.child([CODE/'pregrasp_raised_trial.py','--run','--segmented-high',
            '--simulation-contact-trial','--bedroom-side-table-grasp','--motion-profile','balanced_v1',
            '--efficient-transfer','--reviewed-map-pose',*state['poses']['map'],
            '--measured-contact-start-lift',lift,'--contact-finish','lift','--holding-opening',opening],'lift')
        completed=time.time_ns();report['stage']='verify_current_depth_follow';save()
        c0=transform_point(before_sample['snapshot'],before_geometry['center_m'])
        p0=transform_point(before_sample['snapshot'],before_sample['snapshot']['palm_base_at_depth'])
        accepted=[];rejected=[];end=time.monotonic()+8
        while len(accepted)<3 and time.monotonic()<end:
            sample=runtime.sample();snap=sample['snapshot']
            if min(snap['streams'][key]['stamp_ns'] for key in ('rgb','depth'))<=completed:continue
            try:
                p1=transform_point(snap,snap['palm_base_at_depth'])
                expected=transform_point(snap,c0+p1-p0,inverse=True)
                measured=measure_near(sample['depth'],snap,expected,reference['radius_m'],reference['height_m'])
                proof=lift_follow(before_geometry,before_sample['snapshot'],measured,snap,opening)
                fresh(snap,runtime.node.get_clock().now().nanoseconds,reference['epoch'])
                if accepted and proof['stamp_ns']-accepted[-1]['stamp_ns']<200_000_000:continue
                path=runtime.save(sample,f'after-lift-{len(accepted):02d}',measured)
                accepted.append(dict(proof,capture=str(path)))
            except ValueError as exc:
                path=runtime.save(sample,f'after-lift-rejected-{len(rejected):02d}')
                rejected.append(dict(reason=str(exc),capture=str(path)))
        report.update(lift_verification=accepted,rejected_measurements=rejected,held_opening_rad=opening)
        if len(accepted)!=3:raise ValueError('can_lift_follow_not_verified')
        relatives=np.array([r['center_minus_palm_odom_m'] for r in accepted])
        if np.max(np.linalg.norm(relatives-relatives.mean(0),axis=1))>.02:raise ValueError('can_not_stable_with_palm')
        report.update(status='can_grasp_and_lift_verified',grasp_success_verified=True,can_remains_held=True,arm_retracted=False)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped_requires_current_state_review',reason=str(exc) or 'interrupted',hand_released=False)
    finally:
        try:report['last_fresh_robot_state']=runtime.state()
        except Exception as exc:report['state_error']=str(exc)
        runtime.shutdown();rclpy.shutdown();save();print(json.dumps(report),flush=True)
if not report['grasp_success_verified']:raise SystemExit(1)

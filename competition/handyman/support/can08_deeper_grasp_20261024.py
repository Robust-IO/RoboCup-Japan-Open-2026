"""User-directed new deeper-seat grasp after auditing the failed run08 closure.

Open wider once, advance4cm once, close once, then verify one10cm lift.
Original failed run08 remains failed. No carry, place or protocol is sent here.
"""
import hashlib,json,math,sys,time
from pathlib import Path
import numpy as np
CODE=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,check_inputs,write_json
from canned_grasp_runtime import CanRuntime,TOPICS
from canned_grasp_geometry import fresh,measure_near,transform_point,lift_follow
from held_can_appearance import build_appearance
from gripper_close_gate import CloseGate
from task_outcome_policy import pose_matches
from reviewed_carry_adjustment import endpoint
from motion_speed import finite_duration
import rclpy
base=Path('/__HANDYMAN_PACKAGE__/runtime');old=base/'can-auto-live-20261024-08';read=lambda p:json.loads(p.read_text())
prior=read(old/'grasp/result.json');meta=read(old/'run-meta.json');audit=read(base/'can08-current-state-audit-20261024-01/result.json')
if (prior['stage']!='one_closure' or prior['reason']!='can_closure_inconclusive'
        or prior.get('lift') or meta['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()):
    raise ValueError('audited_same_scene_failed_closure_required')
root=base/'can08-deeper-seat-live-20261024-01';root.mkdir(exist_ok=False);grasp=root/'grasp';grasp.mkdir()
epoch=root.name+'-rgbd';ref=dict(prior['reference'],task_id='grasp',epoch=epoch)
failed_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (old/'result.json',old/'grasp/result.json')}
write_json(root/'run-meta.json',dict(run_id=root.name,mode='live',object='canned_juice',boot_id=meta['boot_id'],
    run_root=str(root),started_ns=time.time_ns(),run_kind='user_reviewed_new_deeper_seat_grasp',prior_failed_run=str(old)))
report=dict(status='starting',task_id='grasp',epoch=epoch,reference=ref,grasp_plan=prior['grasp_plan'],
    run_kind='user_reviewed_new_deeper_seat_grasp',prior_failed_run=str(old),
    user_requested_more_forward_seating=True,wider_opening_rad=.85,extra_forward_m=.04,
    close_commands_maximum=1,automatic_action_retry=False,grasp_success_verified=False,physical_transfer_verified=False,
    task_finished_sent=False,hand_released_by_cleanup=False)
def save():write_json(grasp/'result.json',report)
with RobotLock(),(grasp/'journal.jsonl').open('x') as journal:
    def record(event):
        row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    def stage(name):report['stage']=name;save();record(dict(event='stage',stage=name))
    rclpy.init();rt=CanRuntime(grasp,epoch,record)
    def observe(expected_odom):
        for _ in range(5):
            sample=rt.sample();snap=sample['snapshot'];expected=transform_point(snap,expected_odom,inverse=True)
            try:
                m=measure_near(sample['depth'],snap,expected,ref['radius_m'],ref['height_m'],rgb=sample['rgb'],appearance=ref['source_appearance'])
                fresh(snap,rt.node.get_clock().now().nanoseconds,epoch);return sample,m
            except ValueError as exc:reason=str(exc)
        raise ValueError('current_can_depth_required:'+reason)
    try:
        cfg=read(CODE/'config/apple_transfer_auto_v1.json');write_json(grasp/'input-audit.json',check_inputs(cfg))
        sample,before=observe(ref['center_odom_m']);state=rt.state();last=audit['current_state']
        if (not pose_matches(state['joints'],last['joints']) or math.dist(state['poses']['odom'][:2],last['poses']['odom'][:2])>.02
                or abs(state['poses']['odom'][2]-last['poses']['odom'][2])>.03):raise ValueError('current_scene_changed_since_audit')
        arm=state['joints'][:5];rt.save(sample,'before-user-directed-open',before)
        stage('one_wider_open_to_clear_fingers');topic=TOPICS[3];pub=rt.publisher(topic)
        try:
            record(dict(event='reviewed_wider_open_intent',opening_rad=.85))
            pub.publish(rt.trajectory(['hand_motor_joint'],[.85],6.));record(dict(event='reviewed_wider_open_sent'))
            deadline=time.monotonic()+6.8
            while time.monotonic()<deadline:
                rt.pump(.02);current=rt.state();rt.ownership(topic)
                if not pose_matches(current['joints'][:5]+[0.],arm+[0.]):raise ValueError('arm_moved_during_open')
            if abs(rt.state()['joints'][5]-.85)>.01:raise ValueError('wider_open_not_reached')
        finally:rt.node.destroy_publisher(pub)
        sample,opened=observe(ref['center_odom_m']);opened_odom=transform_point(sample['snapshot'],opened['center_m'])
        rt.save(sample,'after-wider-open',opened)
        stage('one_extra_four_cm_forward');state=rt.state();topic=TOPICS[1];pub=rt.publisher(topic)
        try:
            target=endpoint(state['poses']['odom'],[.04,0.,0.]);duration=finite_duration(rt.profile,[.04,0.,0.])
            record(dict(event='deeper_seat_forward_intent',delta=[.04,0.,0.],target_odom=target,duration_s=duration))
            pub.publish(rt.trajectory(['odom_x','odom_y','odom_t'],target,duration));record(dict(event='deeper_seat_forward_sent'))
            deadline=time.monotonic()+duration+1.2
            while time.monotonic()<deadline:
                rt.pump(.02);current=rt.state();rt.ownership(topic)
                if not pose_matches(current['joints'],arm+[.85]):raise ValueError('open_arm_changed_during_advance')
            report['extra_advance']=dict(delta=[.04,0.,0.],final_state=rt.state(),command_count=1)
        finally:rt.node.destroy_publisher(pub)
        stage('one_new_closure_after_deeper_seating');topic=TOPICS[3];pub=rt.publisher(topic)
        try:
            before_sample,before_geometry=observe(opened_odom);snap=before_sample['snapshot'];state=rt.state()
            offset=np.asarray(before_geometry['center_m'])-snap['palm_base_at_depth']
            if abs(offset[1])>.012 or abs(offset[2])>.02:raise ValueError('deeper_seat_lateral_or_height_not_aligned')
            capture=rt.save(before_sample,'preclose',before_geometry);fresh(snap,rt.node.get_clock().now().nanoseconds,epoch)
            gate=CloseGate(state['joints'][5],6.);report['deeper_measured_offset_m']=offset.tolist();save()
            record(dict(event='can_closure_intent',before=str(capture),current_depth_measurement=before_geometry))
            pub.publish(rt.trajectory(['hand_motor_joint'],[0.],6.));record(dict(event='can_closure_sent',before=str(capture),duration_s=6.))
            sent=time.monotonic();last_ns=state['stamp_ns'];decision=None
            while time.monotonic()-sent<11:
                rt.pump(.02);current=rt.state();rt.ownership(topic)
                if current['stamp_ns']<=last_ns:continue
                last_ns=current['stamp_ns'];decision=gate.observe(time.monotonic()-sent,last_ns,current['joints'][5],
                    max(abs(a-b) for a,b in zip(arm,current['joints'][:5])))
                if decision is not None:break
            close=dict(status='passed' if decision and decision['state']=='contact_candidate' else 'no_contact',
                grasp_closure_commanded=True,closure_evidence=decision,before_capture=str(capture))
            report['close']=close;write_json(grasp/'closure-contact.json',close);save()
            if close['status']!='passed':raise ValueError('deeper_closure_not_contact_candidate')
            opening=decision['observed_opening_rad']
            if not .02<opening<=.62:raise ValueError('deeper_opening_needs_independent_review_no_lift')
        finally:rt.node.destroy_publisher(pub)
        stage('verification_lift');state=rt.state();lift=ref['contact_start_lift_m']
        report['lift']=rt.child([CODE/'pregrasp_raised_trial.py','--run','--segmented-high','--simulation-contact-trial',
            '--bedroom-side-table-grasp','--motion-profile','balanced_v1','--efficient-transfer',
            '--reviewed-map-pose',*state['poses']['map'],'--measured-contact-start-lift',lift,
            '--contact-finish','lift','--holding-opening',opening],'lift')
        completed=time.time_ns();stage('verify_actual_can_follows_lift')
        c0=transform_point(before_sample['snapshot'],before_geometry['center_m']);p0=transform_point(before_sample['snapshot'],before_sample['snapshot']['palm_base_at_depth'])
        appearance=build_appearance(before_sample['rgb'],before_sample['depth'],before_sample['snapshot'],before_geometry)
        accepted=[];rejected=[];deadline=time.monotonic()+8
        while len(accepted)<3 and time.monotonic()<deadline:
            sample=rt.sample();snap=sample['snapshot']
            if min(snap['streams'][k]['stamp_ns'] for k in ('rgb','depth'))<=completed:continue
            try:
                palm=transform_point(snap,snap['palm_base_at_depth']);expected=transform_point(snap,c0+palm-p0,inverse=True)
                m=measure_near(sample['depth'],snap,expected,ref['radius_m'],ref['height_m'],rgb=sample['rgb'],appearance=appearance)
                proof=lift_follow(before_geometry,before_sample['snapshot'],m,snap,opening)
                fresh(snap,rt.node.get_clock().now().nanoseconds,epoch)
                if accepted and proof['stamp_ns']-accepted[-1]['stamp_ns']<200_000_000:continue
                path=rt.save(sample,f'after-lift-{len(accepted):02d}',m);accepted.append(dict(proof,capture=str(path)))
            except ValueError as exc:
                rejected.append(str(exc));rt.save(sample,f'after-lift-rejected-{len(rejected):02d}')
        report.update(lift_verification=accepted,rejected_measurements=rejected,held_opening_rad=opening)
        if len(accepted)!=3:raise ValueError('deeper_grasp_lift_not_verified:'+str(rejected[-3:]))
        offsets=np.array([x['center_minus_palm_odom_m'] for x in accepted])
        if np.max(np.linalg.norm(offsets-offsets.mean(0),axis=1))>.02:raise ValueError('deeper_grasp_not_stable_with_palm')
        report.update(status='can_grasp_and_lift_verified',grasp_success_verified=True,can_remains_held=True,arm_retracted=False,
            wider_open_then_deeper_seat_live_verified=True,automatic_full_initial_transfer_verified=False)
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
    finally:
        try:report['last_fresh_robot_state']=rt.state()
        except Exception as exc:report['state_read_error']=str(exc)
        if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h for p,h in failed_hashes.items()):report['original_failure_changed']=True
        rt.shutdown();rclpy.shutdown();save();print(json.dumps(report),flush=True)
if report['status']!='can_grasp_and_lift_verified':raise SystemExit(1)

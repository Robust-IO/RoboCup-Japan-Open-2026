"""Back up/sync exact progress files after current idle audit; no robot actions."""
import hashlib
import json
import math
from pathlib import Path
import sys
import tarfile
import time

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
from apple_transfer_auto_plan import map_pose

name='apple-capture-budget-live-progress-20261018.json'
pointer='D:/handyman-training-runs/'+name
previous={
    'apple-near-geometry-live-progress-20261018.json':'44556842153e93f5cdf1fd41a1a19c732263b27a0b9b1f46834de90b45a2d068',
    'apple-near-geometry-progress-20261018.json':'ded0a27bd30ada567db19ea3748d812610a318fc81a5dc07a42677f04cff2bf4',
    'phase5-restart-checkpoint-20261017.json':'695c5d6e231f5e01dd02e4d073ac8c24be97b2325c2802034aada18f0153413e',
    'apple-transfer-automation-progress-20261017.json':'d1b62074b74801a6709ecbba8210167d2482beff15ceefd0ba4b4ba9874e4cd5',
    'apple-transfer-live-progress-20261017.json':'efd2d633165d13da1a7c11f8a6794433e34b08935090c57b1b1c614bd25fa24f'}
roots=dict(runs=base,runtime=code,training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
with RobotLock():
    for root in roots.values():
        if (root/name).exists():raise ValueError('new_checkpoint_required')
        for old,digest in previous.items():
            if sha(root/old)!=digest:raise ValueError('checkpoint_changed:'+str(root/old))
    full=base/'automatic-apple-r13c-20261018-01'
    run=read(full/'result.json') if (full/'result.json').exists() else None
    post=read(base/'apple-r13c-post-run-20261018-01/result.json')
    snap=read(Path(post['last_capture'])/'snapshot.json');j=snap['joints']
    ready=post['readiness'];okay=run is not None and run['status']=='automatic_transfer_verified'
    if run is not None and run['status']=='running':raise ValueError('final_current_state_required')
    observed=read(base/'apple-capture-budget-r13c-source-readonly-20261018-03-after-alignment/result.json')
    source=full/'source-observe/manifest.json';frames=read(source)['captures'] if source.exists() else []
    geometry=[r for r in frames if r.get('identity_tracking',{}).get('status')=='near_geometry_tracked']
    robot=dict(capture=post['last_capture'],depth_stamp_ns=snap['streams']['depth']['stamp_ns'],
        base_map_pose=map_pose(snap),arm=[j[k] for k in ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint')],
        hand_motor=j['hand_motor_joint'],head=[j[k] for k in ('head_pan_joint','head_tilt_joint')],
        arm_is_tucked=all(abs(j[k]-v)<.005 for k,v in zip(('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint'),(0,0,0,-math.pi/2,0))),
        holding_object=False if ready.get('ready') is True else 'not_proven_by_empty_start_check',
        readiness=ready.get('ready'),readiness_reason=ready.get('reason'),command_publishers=ready.get('command_publishers'),
        stale_after_restart_or_manual_robot_move=True,fresh_state_required_before_next_action=True)
    if okay:robot.update(apple_location='living_room square_low_table, verified by this full run',room='living_room')
    alignment=read(base/'apple-r13c-current-localization-20261018-01/completion.json')
    deployments={r:read(base/('apple-capture-budget-'+r+'-20261018-deployment.json')) for r in ('r13','r13b','r13c')}
    deployments['r14']=read(base/'apple-close-handoff-r14-20261018-deployment.json')
    progress=dict(status='full_automatic_transfer_verified' if okay else 'full_trial_stopped' if run else 'full_trial_not_started',
        recorded_ns=time.time_ns(),branch='feat/handyman-phase5-grasping',revision='close-handoff-r14 over capture-budget-r13c/near-geometry-r12b',
        deployments=deployments,
        changes=['Measured detector/geometry scheduling costs instead of fixed200ms reserve; genuine400ms final checks remain',
            'Same-frame geometry/support computed once; local ROI pixels exactly equal old full-frame predicates',
            'Decode RGB once; same BGR pixels/model/resolution/confidence',
            'Parallel lossless D-drive saves; all writes complete before inference/index',
            'Bounded250ms preference for a new original joint message,30ms publication/consumer headroom; no post-inference joint replacement'],
        recorded_equivalence_benchmark=read(base/'apple-capture-r13b-benchmark-20261018-02/result.json'),
        failed_readonly_runs_preserved=[str(base/(folder+'/result.json')) for folder in
            ('apple-capture-budget-r13-source-readonly-20261018-01','apple-capture-budget-r13b-source-readonly-20261018-01','apple-capture-budget-r13c-source-readonly-20261018-01')],
        source_clock=dict(before_sync_joint_receipt=read(base/'apple-r13c-joint-arrival-20261018.json'),
            user_triggered_unity_should_synchronize=True,after_sync_readonly=read(base/'apple-capture-budget-r13c-source-readonly-20261018-02-after-sync/result.json'),
            threshold_widened=False,historical_stamps_rewritten=False,system_clocks_changed=False,
            caveat='Historical same issue recovered after Should Synchronize. No proof that all pre-sync age was transport or pure clock skew. Bridge image/TF receipt restamping remains.'),
        localization=dict(audit=read(base/'apple-r13c-bedroom-localization-audit-20261018.json'),completion=alignment,
            previous_anchor5371_stopped=True,current_anchor_pid=alignment['new_anchor_pid'],
            fresh_three_scan_check_required=True,continuous_laser_localization=False,
            warning='Current-scene measurement only; never restore numeric anchor after Unity restart/manual relocation'),
        after_alignment_source_readonly=observed,full_run=run,full_directory=str(full),
        source_navigation=read(full/'source-navigation.json') if (full/'source-navigation.json').exists() else None,
        destination_navigation=read(full/'destination-navigation.json') if (full/'destination-navigation.json').exists() else None,
        live_near_geometry=dict(frames=len(geometry),first=geometry[0]['capture'] if geometry else None,
            reached=bool(geometry),whole_transfer_verified=okay),
        current_robot=robot,post_run_audit=post,
        placement_verification=read(full/'placement-verification.json') if (full/'placement-verification.json').exists() else None,
        uninterrupted_full_transfer_verified=okay,model_or_map_or_unity_files_changed=False,
        navigation_tolerances_unchanged=[.06,.08,.06],placement_mode='single_unlimited',edge_margin_m=.03,edge_uncertainty_m=.01,
        task_finished_sent=False,does_not_exist_sent=False,git_commit_or_push_performed=False,
        scope='One reviewed simulated LayoutA apple transfer; NOT generic30-class grasp/place or competition success-rate acceptance',
        next='Read this checkpoint FIRST. Verify current robot state and fresh observations; never replay an old action/journal or restore old anchor. '+
            ('Next repeat the full workflow from a genuinely new initial scene after fresh localization and source synchronization.' if okay else
             'r13c reached live no-YOLO near geometry, then old forward-only staging check rejected an already closable endpoint. R14 fixes scheduling only and passes tests, but has NOT been live-tested. Robot currently EXTENDED/OPEN at source; do NOT launch the full empty-tucked entry or replay old approach. First fresh near-target audit and an explicitly new reviewed continuation/recovery, then new full regression.'))
    progress['close_handoff_fix']=dict(revision='r14',deployed=True,live_tested=False,
        last_recorded_near_frame_target_base_m=[.7027987227410115,.07279410526853293,.538142458345037],
        last_recorded_near_frame_close_error_xyz_m=[.0135035042410115,-.00520589473146707,.0084037300675673],
        exact_loop_rejection_input_was_not_logged=True,
        original_close_limits_unchanged_m=[.020,.012,.020],
        meaning='Check existing close region before demanding another coarse forward step. Still reacquire/refit/validate at actual close publication.',
        grasp_closure_sent_in_r13c=False,lift_or_release_sent_in_r13c=False)
    robot['holding_object']=False if abs(j['hand_motor_joint']-.65)<.005 else robot['holding_object']
    if not okay:
        robot.update(room='bedroom',apple_location='Still visually on source white_side_table; r13c did NOT close/lift/release; confirm current RGB before action')
    # Keep the benchmark inspectable but avoid duplicating all pixel-result rows.
    progress['recorded_equivalence_benchmark']={k:v for k,v in progress['recorded_equivalence_benchmark'].items() if k not in ('measurements','saving')}
    progress['source_clock']['before_sync_joint_receipt'].pop('rows',None)
    updates={name:progress}
    for old in previous:
        value=read(base/old);value['latest_validation_checkpoint']=pointer
        value['latest_live_validation_checkpoint']=pointer
        value['capture_budget_r13c_latest_status']=progress['status']
        value['next']=progress['next']
        if old in ('phase5-restart-checkpoint-20261017.json','apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json'):
            value.update(current_robot=robot,current='Read latest_validation_checkpoint FIRST: '+progress['status'],
                current_state_authoritative_checkpoint=pointer,
                older_current_fields_are_historical=True)
        if old=='phase5-restart-checkpoint-20261017.json':
            value['navigation_enabled']=False
            value['navigation_state']='R13c source navigation PASSED; reset/restored. Stopped before closure; current arm EXTENDED/OPEN. Read canonical checkpoint.'
            services=value.setdefault('services_at_last_record',{})
            services.update(active_map_anchor_pid=alignment['new_anchor_pid'],
                active_map_anchor='r13c independently measured current BEDROOM laser-map alignment',
                active_map_anchor_record='D:/handyman-training-runs/apple-r13c-current-localization-20261018-01/corrected-map-anchor.json')
        updates[old]=value
    backup=base/'apple-before-capture-budget-live-20261018-checkpoints.tar.gz'
    with tarfile.open(backup,'x:gz') as archive:
        for label,root in roots.items():
            for old in previous:archive.add(root/old,arcname=label+'/'+old)
    encoded={k:json.dumps(v,indent=2,allow_nan=False)+'\n' for k,v in updates.items()}
    for root in roots.values():
        for filename,data in encoded.items():(root/filename).write_text(data)
    hashes={filename:sha(base/filename) for filename in updates}
    if any(sha(root/filename)!=digest for root in roots.values() for filename,digest in hashes.items()):raise ValueError('progress_sync_mismatch')
    report=dict(status='capture_budget_progress_synced',copies=4,hashes=hashes,backup=str(backup),
        full_status=run['status'] if run else None,current_capture=post['last_capture'],robot_commands_sent=False)
    with (base/'apple-capture-budget-live-20261018-checkpoint-sync.json').open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report),flush=True)

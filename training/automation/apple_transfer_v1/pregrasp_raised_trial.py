"""Explicit staged simulation trials. Closure contact requires separate lift evidence."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from tf2_ros import Buffer,TransformListener
from fresh_feedback_wait import wait_for_fresh
from gripper_close_gate import CloseGate
from motion_speed import arm_duration, duration_fields, load_profile
from task_outcome_policy import (pose_matches, hand_is_open, carry_start_from_current,
                                 arm_base_stability, carry_lower_from_current)
from transfer_efficiency import wait_ready, coalesce_wrist_flex

NAMES=['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']


def expected_palm(p):
    lift,flex,_,wrist,_,_=p;a=-flex;b=-flex-wrist
    return [.141+math.cos(a)*.005102036+math.sin(a)*.34923768+math.cos(b)*.012+math.sin(b)*.1405,
            .078,.339254677+lift-math.sin(a)*.005102036+math.cos(a)*.34923768-math.sin(b)*.012+math.cos(b)*.1405]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',action='store_true');parser.add_argument('--resume-lifted',action='store_true');parser.add_argument('--resume-opened',action='store_true');parser.add_argument('--camera-lift-check',action='store_true');parser.add_argument('--segmented-high',action='store_true');parser.add_argument('--retract-high',action='store_true');parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--resume-retract-lower',action='store_true')
    parser.add_argument('--simulation-contact-trial',action='store_true')
    parser.add_argument('--reviewed-map-pose',type=float,nargs=3)
    parser.add_argument('--descend-high',type=float,help='Explicit simulated descent from completed high pose, meters, at most .12')
    parser.add_argument('--descent-start-lift',type=float,default=.4428021848314039)
    parser.add_argument('--contact-finish',choices=['close','lift','carry'])
    parser.add_argument('--carry-lower-from-current',action='store_true',
                        help='Explicit fresh-state completion: already folded held arm; lower only, never re-fold or release')
    parser.add_argument('--measured-contact-start-lift',type=float,help='Measured white-table contact pose; head tilt -.5')
    parser.add_argument('--identity-recording',type=Path);parser.add_argument('--tracking-epoch');parser.add_argument('--tracking-task-id')
    parser.add_argument('--holding-opening',type=float,default=0.)
    parser.add_argument('--place-stage',choices=['extend','lower','release','raise','retract'])
    parser.add_argument('--tucked-camera-check',choices=['raise','restore'],help='Empty tucked arm only: 4 cm lift for hand projection audit')
    parser.add_argument('--prepare-empty-horizontal',action='store_true',help='Open empty hand: lift high, set wrist, extend arm without descent')
    parser.add_argument('--horizontal-observe',choices=['raise','restore','fold','low-view'],help='Explicit empty-arm observation trials')
    parser.add_argument('--white-table-prepare-lift',type=float,help='Reviewed empty-hand horizontal preparation; lift in [.24,.30], head tilt -.5')
    parser.add_argument('--place-palm-height',type=float,default=.60)
    parser.add_argument('--bedroom-side-table-place',action='store_true',help='Explicit LayoutA bedroom side-table experiment, head tilt -.5; no change to other trials')
    parser.add_argument('--bedroom-side-table-grasp',action='store_true',help='Same measured grasp sequence, at reviewed bedroom table rather than living room')
    parser.add_argument('--prepare-hand-already-open',action='store_true',help='Measured empty preparation starts at open .65, not closed zero')
    parser.add_argument('--inverse-place-stage',choices=['prepare','release','raise','retract'],help='Square table: prepare measured horizontal height outside, THEN approach separately')
    parser.add_argument('--release-recording',type=Path);parser.add_argument('--release-plan',type=Path)
    parser.add_argument('--release-epoch');parser.add_argument('--release-task-id')
    parser.add_argument('--feedback-only',action='store_true',help='Read-only 20 second feedback audit; creates no command publishers')
    parser.add_argument('--motion-profile',choices=['baseline','balanced_v1'],default='baseline')
    parser.add_argument('--efficient-transfer',action='store_true')
    parser.add_argument('--coherent-can-startup',action='store_true',
                        help='Can adapter only: retain a still-fresh observed joint-time TF during startup')
    args=parser.parse_args()
    functional_transfer=bool(args.inverse_place_stage or args.bedroom_side_table_grasp)
    if args.coherent_can_startup and not (args.efficient_transfer and args.simulation_contact_trial and functional_transfer):
        raise ValueError('coherent_can_startup_requires_explicit_measured_can_adapter')
    if args.efficient_transfer and not functional_transfer:
        raise ValueError('efficient_mode_requires_measured_transfer')
    if args.carry_lower_from_current and not (args.contact_finish=='carry' and args.bedroom_side_table_grasp
                                             and args.measured_contact_start_lift is not None):
        raise ValueError('exclusive_measured_carry_completion_required')
    carrying_fold=bool(args.contact_finish=='carry' and args.bedroom_side_table_grasp
                       and args.measured_contact_start_lift is not None and .02<args.holding_opening<=.62)
    speed_profile=load_profile(args.motion_profile)
    release_gate_args=(args.release_recording,args.release_plan,args.release_epoch,args.release_task_id)
    if any(release_gate_args) and (not all(release_gate_args) or args.inverse_place_stage!='release'):
        raise ValueError('release_lease_requires_exclusive_inverse_release')
    inverse_plan=None
    if args.inverse_place_stage:
        if not args.simulation_contact_trial or any((args.place_stage,args.contact_finish,args.white_table_prepare_lift is not None,args.measured_contact_start_lift is not None,args.horizontal_observe,args.prepare_empty_horizontal,args.tucked_camera_check,args.retract_high,args.descend_high is not None,args.bedroom_side_table_place,args.bedroom_side_table_grasp,args.feedback_only)):raise ValueError('exclusive_inverse_table_place_required')
        from inverse_table_place_plan import build
        inverse_plan=build(args.inverse_place_stage,args.place_palm_height,args.holding_opening)
    if args.measured_contact_start_lift is not None:
        if (not args.simulation_contact_trial or args.contact_finish not in ('close','lift','carry')
                or not math.isfinite(args.measured_contact_start_lift) or not .24<=args.measured_contact_start_lift<=.30
                or args.white_table_prepare_lift is not None or args.place_stage or args.horizontal_observe
                or args.prepare_empty_horizontal or args.tucked_camera_check or args.descend_high is not None):raise ValueError('Exclusive measured contact trial required')
        if args.contact_finish=='close' and not all((args.identity_recording,args.tracking_epoch,args.tracking_task_id)):
            raise ValueError('Current closure identity evidence required')
    if args.white_table_prepare_lift is not None:
        if (not args.simulation_contact_trial or not math.isfinite(args.white_table_prepare_lift)
                or not .24<=args.white_table_prepare_lift<=.30
                or args.horizontal_observe or args.prepare_empty_horizontal or args.place_stage
                or args.contact_finish or args.descend_high is not None or args.holding_opening
                or args.tucked_camera_check or args.feedback_only):raise ValueError('Choose exclusive reviewed white table preparation')
    if args.horizontal_observe and (not args.simulation_contact_trial or args.prepare_empty_horizontal or args.place_stage or args.contact_finish or args.descend_high is not None or args.holding_opening or args.tucked_camera_check):raise ValueError('Observation pose requires exclusive empty high trial')
    if args.prepare_empty_horizontal and (not args.simulation_contact_trial or args.place_stage or args.contact_finish or args.descend_high is not None or args.holding_opening or args.tucked_camera_check):raise ValueError('Empty preparation requires exclusive simulation trial')
    if args.tucked_camera_check and (not args.simulation_contact_trial or args.place_stage or args.contact_finish or args.descend_high is not None or args.holding_opening):raise ValueError('Camera audit requires exclusive empty-hand simulation trial')
    if args.place_stage and (not args.simulation_contact_trial or args.contact_finish or args.descend_high is not None):raise ValueError('Placement requires exclusive simulation trial')
    if args.bedroom_side_table_place and not args.place_stage:raise ValueError('Bedroom override requires placement stage')
    if args.bedroom_side_table_grasp and (args.bedroom_side_table_place or not (args.white_table_prepare_lift is not None or args.measured_contact_start_lift is not None)):raise ValueError('Bedroom grasp requires exclusive measured grasp stage')
    if args.prepare_hand_already_open and args.white_table_prepare_lift is None:raise ValueError('Open start only for measured empty preparation')
    if not math.isfinite(args.place_palm_height) or not .35<=args.place_palm_height<=.72:raise ValueError('Invalid placement palm height')
    if args.contact_finish and (not args.simulation_contact_trial or args.descend_high is not None):raise ValueError('Finish requires exclusive simulation contact trial')
    if not math.isfinite(args.holding_opening) or not 0<=args.holding_opening<=.65:raise ValueError('Invalid observed holding opening')
    if args.holding_opening and args.contact_finish not in ('lift','carry') and not args.place_stage and not args.inverse_place_stage:raise ValueError('Holding opening only for lift/carry/place')
    if args.descend_high is not None and (not args.simulation_contact_trial or not math.isfinite(args.descend_high) or not 0<args.descend_high<=.12):raise ValueError('Descent requires explicit contact trial and distance in (0,.12]')
    if not math.isfinite(args.descent_start_lift) or not .26<=args.descent_start_lift<=.443:raise ValueError('Invalid descent start')
    if args.descend_high is not None and args.descent_start_lift-args.descend_high<.26:raise ValueError('Descent below experiment lower bound')
    if args.simulation_contact_trial:
        if os.environ.get('ROS_DOMAIN_ID')!='71' or not args.segmented_high or args.retract_high or args.reviewed_map_pose is None or not all(math.isfinite(v) for v in args.reviewed_map_pose):raise ValueError('Explicit simulation pose and segmented-high required')
        if args.inverse_place_stage or args.bedroom_side_table_grasp:
            # Measured transfer uses source/destination geometry and current
            # pose/identity/release checks, not a historical world-coordinate
            # rectangle. All live TF, reach, posture and ownership gates remain.
            pass
        elif args.bedroom_side_table_place:
            if not (9.4<=args.reviewed_map_pose[0]<=10.4 and -6.95<=args.reviewed_map_pose[1]<=-5.8 and abs(args.reviewed_map_pose[2])<.35):raise ValueError('Outside reviewed bedroom side-table region')
        elif args.place_stage or args.tucked_camera_check or args.prepare_empty_horizontal or args.horizontal_observe:
            if not (1.5<=args.reviewed_map_pose[0]<=2.6 and 2.5<=args.reviewed_map_pose[1]<=3.3):raise ValueError('Outside square table experiment region')
        elif math.dist(args.reviewed_map_pose[:2],(3.1743,.3133))>.4:raise ValueError('Outside table experiment region')
    elif args.reviewed_map_pose is not None:raise ValueError('Pose override requires explicit simulation trial')
    if args.resume_retract_lower and not args.retract_high:raise ValueError('Lower resume requires --retract-high')
    retract_start=2 if args.resume_retract_lower else 0
    # Resume only the independently screened final lowering stage.
    # Its exact starting joints are checked again before publishing.
    if args.retract_high:
        if args.segmented_high or args.resume_lifted or args.resume_opened or args.camera_lift_check:raise ValueError('Choose only retract-high')
        args.segmented_high=True  # Reuse stricter feedback and base gates.
    if args.segmented_high and (args.resume_lifted or args.resume_opened or args.camera_lift_check):raise ValueError('Segmented trial starts tucked, no resume flags')
    if args.camera_lift_check and (args.resume_lifted or args.resume_opened):raise ValueError('Camera check requires extended starting pose')
    if args.resume_lifted and args.resume_opened:raise ValueError('Choose one resume state')
    if args.feedback_only and args.run:raise ValueError('Read-only mode cannot use --run')
    if not args.run and not args.feedback_only:raise SystemExit('No movement without --run')
    args.output.mkdir(exist_ok=False)
    rclpy.init();node=rclpy.create_node('phase5_raised_pregrasp_trial');buf=Buffer();listener=TransformListener(buf,node)
    latest={};events=[];pubs={};base_anchor=None;feedback_epoch=0
    def cb(m):
        if len(m.name)!=len(m.position) or any(m.name.count(n)!=1 for n in NAMES):return
        p=[float(m.position[m.name.index(n)]) for n in NAMES]
        if not all(math.isfinite(v) for v in p):return
        latest.update(p=p,stamp=Time.from_msg(m.header.stamp),recv=time.monotonic())
        latest['head']=[float(m.position[m.name.index(n)]) if m.name.count(n)==1 else float('nan') for n in ('head_pan_joint','head_tilt_joint')]
    sub=node.create_subscription(JointState,'/hsrb/joint_states',cb,qos_profile_sensor_data)
    topics={'arm':'/hsrb/arm_trajectory_controller/command','hand':'/hsrb/gripper_controller/command'}
    def check_fresh():
        if not latest or time.monotonic()-latest['recv']>.4:raise RuntimeError('stale_joints')
        if not -.02<=(node.get_clock().now()-latest['stamp']).nanoseconds/1e9<=.4:raise RuntimeError('stale_capture')
        head_tilt=-.5 if args.white_table_prepare_lift is not None or args.measured_contact_start_lift is not None or args.bedroom_side_table_place or args.inverse_place_stage else (-.8 if args.simulation_contact_trial else -.5)
        if args.segmented_high and not args.release_recording and any(not math.isfinite(v) or abs(v-g)>.02 for v,g in zip(latest['head'],(0.,head_tilt))):raise RuntimeError('head_pose_not_screened')
        if args.simulation_contact_trial:
            for topic in ('/hsrb/command_velocity','/hsrb/omni_base_controller/command','/hsrb/head_trajectory_controller/command'):
                if node.count_publishers(topic):raise RuntimeError('other_motion_controller_present')
        tf=buf.lookup_transform('map','base_footprint',Time());t=tf.transform.translation;q=tf.transform.rotation
        age=(node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
        if age < -.02:raise RuntimeError('future_base_tf')
        if age > .4:raise RuntimeError('stale_base_tf')
        pose=(t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        if base_anchor is not None:
            stability=arm_base_stability(base_anchor,pose,carrying=carrying_fold)
            previous=report.get('base_motion',{})
            stability['maximum_translation_m']=max(stability['translation_m'],previous.get('maximum_translation_m',0.))
            stability['maximum_yaw_rad']=max(stability['yaw_rad'],previous.get('maximum_yaw_rad',0.))
            report['base_motion']=stability
            if not stability['allowed']:raise RuntimeError('base_moved')
        return pose
    def fresh():
        def on_wait():
            nonlocal feedback_epoch
            feedback_epoch+=1
            events.append(dict(event='feedback_wait',epoch=feedback_epoch))
        return wait_for_fresh(check_fresh,lambda dt:rclpy.spin_once(node,timeout_sec=dt),on_wait)
    def verify_tf(goal):
        if args.release_recording:
            # Hand-only release uses actual image-time palm/target TF in the
            # publication lease; the approximate2D arm model is not a second veto.
            events.append(dict(event='analytic_palm_model_not_required_for_leased_hand_only_release'))
            return
        requested=latest['stamp'];deadline=time.monotonic()+1.0
        while not buf.can_transform('base_link','hand_palm_link',requested):
            if time.monotonic()>=deadline:raise RuntimeError('image_time_tf_wait_expired')
            rclpy.spin_once(node,timeout_sec=.03);fresh()
        tf=buf.lookup_transform('base_link','hand_palm_link',requested);t=tf.transform.translation
        actual=[t.x,t.y,t.z];expected=expected_palm(goal);error=math.dist(actual,expected)
        events.append(dict(event='palm_tf',actual=actual,expected=expected,error_m=error))
        if error>.015:raise RuntimeError('palm_model_disagrees_with_tf')
    def move(kind,goal):
        fresh()
        for k,t in topics.items():
            if node.count_subscribers(t)!=1 or node.count_publishers(t)!=1:raise RuntimeError('controller_ownership_changed')
        msg=JointTrajectory();msg.joint_names=NAMES[:5] if kind=='arm' else NAMES[5:]
        if kind=='hand' and args.contact_finish=='close' and args.measured_contact_start_lift is not None:
            from apple_approach_lease import read_latest
            deadline=time.monotonic()+3.
            while True:
                fresh()
                tf=buf.lookup_transform('odom','base_footprint',Time());t=tf.transform.translation;q=tf.transform.rotation
                tf_age=(node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
                if not -.02<=tf_age<=.4:raise RuntimeError('stale_odom_for_closure')
                odom=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
                try:
                    report['closure_identity_evidence']=read_latest(args.identity_recording,closure=True,
                        now_ns=node.get_clock().now().nanoseconds,epoch=args.tracking_epoch,task_id=args.tracking_task_id,
                        current_odom=odom,palm_height=expected_palm(latest['p'])[2],expected_room='bedroom' if args.bedroom_side_table_grasp else 'living_room')
                    break
                except (ValueError,FileNotFoundError) as exc:
                    retryable=str(exc) in ('vision_evidence_expired_or_unsynchronised','current_identity_not_confirmed','vision_result_not_fresh','no_current_identity_record') or isinstance(exc,(json.JSONDecodeError,FileNotFoundError))
                    if not retryable or time.monotonic()>=deadline:raise
                    rclpy.spin_once(node,timeout_sec=.01)
        duration=arm_duration(speed_profile,kind,latest['p'],goal,
            holding=args.holding_opening>.02,segmented=args.segmented_high)
        close_gate=CloseGate(latest['p'][5],duration) if kind=='hand' and args.contact_finish=='close' else None
        pt=JointTrajectoryPoint();pt.positions=goal[:5] if kind=='arm' else goal[5:]
        pt.time_from_start.sec,pt.time_from_start.nanosec=duration_fields(duration);msg.points=[pt]
        if kind=='hand' and args.release_recording:
            from table_release_gate import read_latest as read_release
            deadline=time.monotonic()+3.
            while True:
                pose=fresh()
                try:
                    report['release_geometry_evidence']=read_release(args.release_recording,args.release_plan,
                        now_ns=node.get_clock().now().nanoseconds,epoch=args.release_epoch,task_id=args.release_task_id,
                        current_map_pose=pose,current_joints=latest['p'])
                    if not 0<=(node.get_clock().now().nanoseconds-report['release_geometry_evidence']['source_stamp_ns'])/1e9<=.4:
                        raise ValueError('fresh_synchronised_rgbd_required')
                    break
                except (ValueError,FileNotFoundError) as exc:
                    retryable=isinstance(exc,(FileNotFoundError,json.JSONDecodeError)) or str(exc) in ('fresh_synchronised_rgbd_required','fresh_joint_snapshot_required','no_current_release_capture')
                    if not retryable or time.monotonic()>=deadline:raise
                    rclpy.spin_once(node,timeout_sec=.01)
        pubs[kind].publish(msg)
        if kind=='hand' and args.contact_finish=='close':report['grasp_closure_commanded']=True
        events.append(dict(event='command',kind=kind,positions=goal.copy(),duration_s=duration,motion_profile=args.motion_profile));print(events[-1],flush=True)
        sent=time.monotonic();deadline=sent+duration+5;start=None;last=latest['stamp'].nanoseconds
        while time.monotonic()<deadline:
            epoch=feedback_epoch
            rclpy.spin_once(node,timeout_sec=.05);fresh()
            if epoch!=feedback_epoch:
                start=None  # Missing feedback cannot count as settled time.
                if close_gate:close_gate.reset_window()
            if time.monotonic()>=deadline:break
            if latest['stamp'].nanoseconds<=last:continue
            last=latest['stamp'].nanoseconds
            if close_gate:
                decision=close_gate.observe(time.monotonic()-sent,last,latest['p'][5],max(abs(x-y) for x,y in zip(goal[:5],latest['p'][:5])))
                if decision is None:continue
                report['closure_evidence']=decision
                events.append(dict(event='closure_evidence',**decision))
                if decision['state']!='contact_candidate':raise RuntimeError(decision['state'])
                verify_tf(latest['p']);return
            reached=(hand_is_open(latest['p'][5]) if kind=='hand' and args.release_recording else
                     pose_matches(latest['p'],goal,endpoint=True) if functional_transfer else
                     max(abs(x-y) for x,y in zip(goal,latest['p']))<(.002 if args.segmented_high else .005))
            if reached:
                if start is None:start=time.monotonic()
                if time.monotonic()-start>.6 and (not args.segmented_high or time.monotonic()-sent>=duration):
                    verify_tf(latest['p'] if functional_transfer else goal);return
            else:start=None
        raise RuntimeError('closure_unstable_or_insufficient_feedback' if close_gate else 'stage_not_settled')
    def execute_steps(initial, steps):
        if args.efficient_transfer:
            steps=coalesce_wrist_flex(initial,steps)
        for step in steps:
            if step.get('coordinated_wrist_flex'):
                events.append(dict(event='coordinated_wrist_flex',lift_unchanged=True,
                                   grip_unchanged=True,speed_limits_unchanged=True))
            move(step['kind'],step['goal'])
    report={'grasp_closure_commanded':False,'efficient_non_navigation':args.efficient_transfer}
    try:
        if args.efficient_transfer:
            if args.coherent_can_startup:
                from can_arm_startup import CurrentTransformReadiness
                startup_transform=CurrentTransformReadiness()
            def feedback_ready():
                if not latest:return False
                if (time.monotonic()-latest['recv']>.4
                        or not -.02<=(node.get_clock().now()-latest['stamp']).nanoseconds/1e9<=.4):return False
                if not all(node.count_subscribers(t)==1 for t in topics.values()):return False
                map_ready=buf.can_transform('map','base_footprint',Time())
                if args.coherent_can_startup:
                    return map_ready and startup_transform.ready(latest['stamp'].nanoseconds,
                        node.get_clock().now().nanoseconds,
                        lambda ns:buf.can_transform('base_link','hand_palm_link',Time(nanoseconds=ns)))
                return map_ready and buf.can_transform('base_link','hand_palm_link',latest['stamp'])
            wait_ready(feedback_ready,lambda dt:rclpy.spin_once(node,timeout_sec=dt))
            if args.coherent_can_startup:
                events.append(dict(event='fresh_coherent_can_startup_tf',stamp_ns=startup_transform.stamp_ns,
                    latest_joint_ns=latest['stamp'].nanoseconds,latest_joint_state_not_replaced=True))
        else:
            deadline=time.monotonic()+4
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        base_anchor=fresh()
        if args.feedback_only:
            deadline=time.monotonic()+20;samples=0
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02);fresh();samples+=1
            report.update(status='passed',mode='feedback_only',samples=samples,waits=feedback_epoch,final_joints=latest['p'])
            return
        if args.simulation_contact_trial:
            if math.dist(base_anchor[:2],args.reviewed_map_pose[:2])>.015 or abs(base_anchor[2]-args.reviewed_map_pose[2])>.02:raise RuntimeError('not_explicit_trial_pose')
        else:
            if math.dist(base_anchor[:2],(3.164,.305))>.05 or abs(base_anchor[2]+1.385)>.05:raise RuntimeError('not_reviewed_staging_location')
            if args.segmented_high and (math.dist(base_anchor[:2],(3.1574,.31426))>.02 or abs(base_anchor[2]+1.37176)>.02):raise RuntimeError('not_20261010_reviewed_pose')
        initial=[0.,0.,0.,-math.pi/2,0.,0.]
        if args.prepare_hand_already_open:initial[5]=.65
        if args.horizontal_observe:
            initial=[.4428021848314039,-1.7,0.,.1292036732051034,0.,.65] if args.horizontal_observe=='raise' else [.4428021848314039,-1.4,0.,-.17079632679489665,0.,.65]
            if args.horizontal_observe=='low-view':initial=[.4428021848314039,0.,0.,.1292036732051034,0.,.65]
        if args.prepare_empty_horizontal:initial[5]=.65
        if args.tucked_camera_check:initial=[.04 if args.tucked_camera_check=='restore' else 0.,0.,0.,-math.pi/2,0.,.65]
        if args.resume_lifted or args.resume_opened:initial[0]=.2509516385
        if args.resume_opened:initial[5]=.65
        if args.camera_lift_check:initial=[.2509516385,-1.2,0.,-.3707963268,0.,.65]
        if args.retract_high:
            from segmented_pregrasp_plan import build_retract_plan
            initial=build_retract_plan()[retract_start]['expected_before']
        if args.descend_high is not None:
            from segmented_pregrasp_plan import build_plan
            initial=build_plan()['stages'][-1]['target'].copy()
            initial[0]=args.descent_start_lift
        if args.contact_finish:
            initial=[.2628021848314039,-1.7,0.,.1292036732051034,0.,.65 if args.contact_finish=='close' else args.holding_opening]
            if args.measured_contact_start_lift is not None:initial[0]=args.measured_contact_start_lift
            if args.contact_finish=='carry':
                initial[0]=args.measured_contact_start_lift+.10 if args.measured_contact_start_lift is not None else .3628021848314039
                if args.bedroom_side_table_grasp and args.measured_contact_start_lift is not None:
                    initial=(carry_lower_from_current(latest['p'],args.holding_opening) if args.carry_lower_from_current else
                             carry_start_from_current(initial,latest['p'],args.measured_contact_start_lift))
        if args.place_stage:
            initial=[.4428021848314039,-1.7,0.,.1292036732051034,0.,args.holding_opening]
            if args.place_stage=='extend':initial=[0.,0.,0.,-math.pi/2,0.,args.holding_opening]
            if args.place_stage in ('release','raise'):initial[0]=.4428021848314039+args.place_palm_height-.72
            if args.place_stage=='raise':initial[5]=.65
            if args.place_stage=='retract':initial[0]=.4428021848314039+args.place_palm_height-.72+.08;initial[5]=.65
        if inverse_plan:
            if args.release_recording:
                from inverse_table_place_plan import release_from_current
                inverse_plan=release_from_current(inverse_plan,latest['p'])
            elif args.inverse_place_stage in ('raise','retract'):
                from inverse_table_place_plan import cleanup_from_current
                inverse_plan=cleanup_from_current(inverse_plan,args.inverse_place_stage,latest['p'])
            initial=inverse_plan['initial'].copy()
        if (not pose_matches(latest['p'],initial) if functional_transfer else
                max(abs(x-y) for x,y in zip(initial,latest['p']))>.02):raise RuntimeError('not_tucked_initial_pose')
        verify_tf(latest['p'] if functional_transfer else initial)
        for k,t in topics.items():
            if node.count_publishers(t)!=0 or node.count_subscribers(t)!=1:raise RuntimeError('control_not_exclusive')
            pubs[k]=node.create_publisher(JointTrajectory,t,10)
        if args.efficient_transfer:
            wait_ready(lambda: all(p.get_subscription_count()==1 and node.count_publishers(topics[k])==1
                       for k,p in pubs.items()),lambda dt:rclpy.spin_once(node,timeout_sec=dt))
        else:
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if inverse_plan:
            if not pose_matches(latest['p'],initial):raise RuntimeError('inverse_place_start_mismatch')
            execute_steps(initial,inverse_plan['steps'])
            report.update(inverse_place_stage=args.inverse_place_stage,placement_success_verified=False,no_base_motion=True)
        elif args.white_table_prepare_lift is not None:
            if (not pose_matches(latest['p'],initial) if functional_transfer else max(abs(x-y) for x,y in zip(initial,latest['p']))>.005):raise RuntimeError('white_table_preparation_start_mismatch')
            goal=initial.copy()
            steps=[]
            for index,value in [(0,args.white_table_prepare_lift),(5,.65),(3,.1292036732051034),(1,-1.7)]:
                goal[index]=value
                if (index==5 and args.prepare_hand_already_open and speed_profile['mode']!='legacy'
                        and abs(latest['p'][5]-.65)<.002):
                    fresh();events.append(dict(event='already_open_no_command',opening=latest['p'][5]))
                else:steps.append(dict(kind='hand' if index==5 else 'arm',goal=goal.copy()))
            execute_steps(initial,steps)
            report['white_table_preparation']=dict(lift=args.white_table_prepare_lift,no_base_motion=True,no_closure=True)
        elif args.horizontal_observe:
            if max(abs(x-y) for x,y in zip(initial,latest['p']))>.005:raise RuntimeError('observation_pose_start_mismatch')
            goal=initial.copy()
            if args.horizontal_observe=='low-view':
                goal[0]=.3;move('arm',goal)
                goal[3]=.32920367320510335;move('arm',goal)
                goal[1]=-1.9;move('arm',goal)
            elif args.horizontal_observe=='raise':
                goal[1]=-1.4;move('arm',goal)
                goal[3]=-.17079632679489665;move('arm',goal)
            elif args.horizontal_observe=='restore':
                goal[3]=.1292036732051034;move('arm',goal)
                goal[1]=-1.7;move('arm',goal)
            else:
                goal[3]=.1292036732051034;move('arm',goal)
                goal[1]=0.;move('arm',goal)
            report['horizontal_observe']=args.horizontal_observe
        elif args.prepare_empty_horizontal:
            if max(abs(x-y) for x,y in zip(initial,latest['p']))>.005:raise RuntimeError('empty_preparation_start_mismatch')
            goal=initial.copy()
            goal[0]=.4428021848314039;move('arm',goal)
            goal[3]=.1292036732051034;move('arm',goal)
            goal[1]=-1.7;move('arm',goal)
            report['empty_horizontal_prepared']=True
        elif args.tucked_camera_check:
            if max(abs(x-y) for x,y in zip(initial,latest['p']))>.005:raise RuntimeError('camera_audit_start_mismatch')
            goal=initial.copy();goal[0]=.04 if args.tucked_camera_check=='raise' else 0.
            move('arm',goal)
            report['tucked_camera_check']=args.tucked_camera_check
        elif args.place_stage:
            if max(abs(x-y) for x,y in zip(initial,latest['p']))>.005:raise RuntimeError('placement_start_mismatch')
            goal=initial.copy()
            if args.place_stage=='extend':
                goal[0]=.4428021848314039;move('arm',goal)
                goal[3]=.1292036732051034;move('arm',goal)
                goal[1]=-1.7;move('arm',goal)
            elif args.place_stage=='lower':
                goal[0]=.4428021848314039+args.place_palm_height-.72;move('arm',goal)
            elif args.place_stage=='release':
                goal[5]=.65;move('hand',goal)
            elif args.place_stage=='retract':
                goal[1]=0.;move('arm',goal)
                goal[3]=-math.pi/2;move('arm',goal)
                goal[0]=0.;move('arm',goal)
            else:
                goal[0]+=.08;move('arm',goal)
            report['place_stage']=args.place_stage
            report['placement_success_verified']=False
        elif args.contact_finish:
            if (not pose_matches(latest['p'],initial) if functional_transfer else max(abs(x-y) for x,y in zip(initial,latest['p']))>.005):raise RuntimeError('finish_start_mismatch')
            goal=initial.copy()
            if args.contact_finish=='carry':
                # Keep grip unchanged. Retract elbow high, restore wrist, then lower.
                steps=[]
                sequence=((0,0.),) if args.carry_lower_from_current else ((1,0.),(3,-math.pi/2),(0,0.))
                for index,value in sequence:
                    goal[index]=value;steps.append(dict(kind='arm',goal=goal.copy()))
                execute_steps(initial,steps)
                report['carry_completion_from_current']=args.carry_lower_from_current
            else:
                if args.contact_finish=='close':goal[5]=0.
                else:goal[0]+=.10
                move('hand' if args.contact_finish=='close' else 'arm',goal)
        elif args.descend_high is not None:
            if max(abs(x-y) for x,y in zip(initial,latest['p']))>.005:raise RuntimeError('descent_start_mismatch')
            lowered=initial.copy();lowered[0]-=args.descend_high
            events.append(dict(event='stage',name='explicit_high_descent',distance_m=args.descend_high))
            move('arm',lowered)
        elif args.segmented_high:
            from segmented_pregrasp_plan import build_plan,build_retract_plan
            for stage in (build_retract_plan()[retract_start:] if args.retract_high else build_plan()['stages']):
                fresh()
                if max(abs(x-y) for x,y in zip(stage['expected_before'],latest['p']))>.005:raise RuntimeError('stage_start_mismatch')
                events.append(dict(event='stage',name=stage['name']))
                move('hand' if stage['changed_joint']=='hand_motor_joint' else 'arm',stage['target'])
        elif args.camera_lift_check:
            # Pure +2 cm vertical translation: unchanged horizontal arm envelope,
            # increased clearance from the previously screened tabletop.
            raised=initial[:];raised[0]+=.02;move('arm',raised)
        else:
            lifted=[.2509516385,0.,0.,-math.pi/2,0.,0.]
            if not (args.resume_lifted or args.resume_opened):move('arm',lifted)
            opened=lifted[:];opened[5]=.65
            if not args.resume_opened:move('hand',opened)
            extended=[.2509516385,-1.2,0.,-.3707963268,0.,.65];move('arm',extended)
        report.update(status='passed',final_joints=latest['p'])
    except Exception as error:report.update(status='failed',reason=str(error))
    finally:
        report.update(events=events,base_anchor=base_anchor,grasp_attempted=report['grasp_closure_commanded'],grasp_success_verified=False,automatic_return=False,simulation_contact_trial=args.simulation_contact_trial,collision_certified=False)
        (args.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
        node.destroy_node();rclpy.shutdown()
    if report['status']!='passed':raise SystemExit(1)


if __name__=='__main__':main()

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
    parser.add_argument('--holding-opening',type=float,default=0.)
    parser.add_argument('--place-stage',choices=['extend','lower','release','raise','retract'])
    parser.add_argument('--place-palm-height',type=float,default=.60)
    parser.add_argument('--feedback-only',action='store_true',help='Read-only 20 second feedback audit; creates no command publishers')
    args=parser.parse_args()
    if args.place_stage and (not args.simulation_contact_trial or args.contact_finish or args.descend_high is not None):raise ValueError('Placement requires exclusive simulation trial')
    if not math.isfinite(args.place_palm_height) or not .35<=args.place_palm_height<=.72:raise ValueError('Invalid placement palm height')
    if args.contact_finish and (not args.simulation_contact_trial or args.descend_high is not None):raise ValueError('Finish requires exclusive simulation contact trial')
    if not math.isfinite(args.holding_opening) or not 0<=args.holding_opening<=.65:raise ValueError('Invalid observed holding opening')
    if args.holding_opening and args.contact_finish not in ('lift','carry') and not args.place_stage:raise ValueError('Holding opening only for lift/carry/place')
    if args.descend_high is not None and (not args.simulation_contact_trial or not math.isfinite(args.descend_high) or not 0<args.descend_high<=.12):raise ValueError('Descent requires explicit contact trial and distance in (0,.12]')
    if not math.isfinite(args.descent_start_lift) or not .26<=args.descent_start_lift<=.443:raise ValueError('Invalid descent start')
    if args.descend_high is not None and args.descent_start_lift-args.descend_high<.26:raise ValueError('Descent below experiment lower bound')
    if args.simulation_contact_trial:
        if os.environ.get('ROS_DOMAIN_ID')!='71' or not args.segmented_high or args.retract_high or args.reviewed_map_pose is None or not all(math.isfinite(v) for v in args.reviewed_map_pose):raise ValueError('Explicit simulation pose and segmented-high required')
        if args.place_stage:
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
        if args.segmented_high and any(not math.isfinite(v) or abs(v-g)>.02 for v,g in zip(latest['head'],(0.,-.8 if args.simulation_contact_trial else -.5))):raise RuntimeError('head_pose_not_screened')
        if args.simulation_contact_trial:
            for topic in ('/hsrb/command_velocity','/hsrb/omni_base_controller/command','/hsrb/head_trajectory_controller/command'):
                if node.count_publishers(topic):raise RuntimeError('other_motion_controller_present')
        tf=buf.lookup_transform('map','base_footprint',Time());t=tf.transform.translation;q=tf.transform.rotation
        age=(node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
        if age < -.02:raise RuntimeError('future_base_tf')
        if age > .4:raise RuntimeError('stale_base_tf')
        pose=(t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        if base_anchor is not None and (math.dist(pose[:2],base_anchor[:2])>.015 or abs(math.atan2(math.sin(pose[2]-base_anchor[2]),math.cos(pose[2]-base_anchor[2])))>.03):raise RuntimeError('base_moved')
        return pose
    def fresh():
        def on_wait():
            nonlocal feedback_epoch
            feedback_epoch+=1
            events.append(dict(event='feedback_wait',epoch=feedback_epoch))
        return wait_for_fresh(check_fresh,lambda dt:rclpy.spin_once(node,timeout_sec=dt),on_wait)
    def verify_tf(goal):
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
        duration=6 if args.segmented_high else 4
        close_gate=CloseGate(latest['p'][5],duration) if kind=='hand' and args.contact_finish=='close' else None
        pt=JointTrajectoryPoint();pt.positions=goal[:5] if kind=='arm' else goal[5:];pt.time_from_start.sec=duration;msg.points=[pt]
        pubs[kind].publish(msg);events.append(dict(event='command',kind=kind,positions=goal.copy(),duration_s=duration));print(events[-1],flush=True)
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
            if max(abs(x-y) for x,y in zip(goal,latest['p']))<(.002 if args.segmented_high else .005):
                if start is None:start=time.monotonic()
                if time.monotonic()-start>.6 and (not args.segmented_high or time.monotonic()-sent>=duration):
                    verify_tf(goal);return
            else:start=None
        raise RuntimeError('closure_unstable_or_insufficient_feedback' if close_gate else 'stage_not_settled')
    report={}
    try:
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
            if args.contact_finish=='carry':initial[0]=.3628021848314039
        if args.place_stage:
            initial=[.4428021848314039,-1.7,0.,.1292036732051034,0.,args.holding_opening]
            if args.place_stage=='extend':initial=[0.,0.,0.,-math.pi/2,0.,args.holding_opening]
            if args.place_stage in ('release','raise'):initial[0]=.4428021848314039+args.place_palm_height-.72
            if args.place_stage=='raise':initial[5]=.65
            if args.place_stage=='retract':initial[0]=.4428021848314039+args.place_palm_height-.72+.08;initial[5]=.65
        if max(abs(x-y) for x,y in zip(initial,latest['p']))>.02:raise RuntimeError('not_tucked_initial_pose')
        verify_tf(initial)
        for k,t in topics.items():
            if node.count_publishers(t)!=0 or node.count_subscribers(t)!=1:raise RuntimeError('control_not_exclusive')
            pubs[k]=node.create_publisher(JointTrajectory,t,10)
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if args.place_stage:
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
            if max(abs(x-y) for x,y in zip(initial,latest['p']))>.005:raise RuntimeError('finish_start_mismatch')
            goal=initial.copy()
            if args.contact_finish=='carry':
                # Keep grip unchanged. Retract elbow high, restore wrist, then lower.
                goal[1]=0.;move('arm',goal)
                goal[3]=-math.pi/2;move('arm',goal)
                goal[0]=0.;move('arm',goal)
            else:
                if args.contact_finish=='close':goal[5]=0.
                else:goal[0]+=.10
                report['grasp_closure_commanded']=args.contact_finish=='close'
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
        report.update(events=events,base_anchor=base_anchor,grasp_attempted=bool(args.contact_finish),grasp_success_verified=False,automatic_return=False,simulation_contact_trial=args.simulation_contact_trial,collision_certified=False)
        (args.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
        node.destroy_node();rclpy.shutdown()
    if report['status']!='passed':raise SystemExit(1)


if __name__=='__main__':main()

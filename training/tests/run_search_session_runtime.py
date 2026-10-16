"""End-to-end lifecycle acceptance. Fake Nav2/TF/diagnostics, real processes/maps."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',choices=('sequence','no_target','cancel','runtime_shutdown','observer_crash','observer_crash_orphan','worker_crash','worker_crash_early','map_change','before_send','delayed_accept','accepted_lost','recovery','recovery_unknown','reject_once','journal_write_failure'),required=True)
    p.add_argument('--vision-log',type=Path,help='Replay recorded diagnostics with synthetic timestamps; domain 73 only')
    p.add_argument('--verify-continuation',action='store_true')
    p.add_argument('--multi-point',action='store_true')
    p.add_argument('--cancel-at-transition',action='store_true')
    p.add_argument('--exhaust-points',action='store_true')
    p.add_argument('--expire-during-second',action='store_true')
    p.add_argument('--auto-head',action='store_true')
    p.add_argument('--active-ring',action='store_true',help='Generated eight-view ring in isolated temporary package only')
    p.add_argument('--bound-ring',action='store_true',help='Real task bundle and runtime binding with synthetic private owner')
    p.add_argument('--replay-bound',action='store_true',help='Restart runtime with same journal and replay consumed task')
    p.add_argument('--session-budget-seconds',type=float)
    p.add_argument('--slow-navigation',action='store_true',help='Simulate 65 s healthy navigation beyond all old short cutoffs')
    p.add_argument('--budget-stop',choices=('event','expiry'))
    p.add_argument('--head-tf-recovery',action='store_true',
                   help='Drop synthetic TF after head-ready, require re-observation without a second motion')
    p.add_argument('--head-skew-ms',type=int,choices=(0,14),default=0,
                   help='Synthetic head-only clock offset; domain 73 only')
    p.add_argument('--room-case',choices=('inside','lobby','boundary'),
                   help='Living-room target ownership regression; synthetic geometry, domain 73 only')
    p.add_argument('--supplement-points',type=int,choices=(0,1,2),default=0)
    p.add_argument('--turn-only',action='store_true',help='Verify per-goal BT selection; fake navigation does not simulate rotation')
    p.add_argument('--turn-tf-delay',action='store_true',help='Delay new-worker TF to reproduce live startup race')
    p.add_argument('--outside-room-ring',action='store_true')
    p.add_argument('--head-fault',choices=('missing_feedback','wrong_angle','cancel_during_motion','stop_feedback_lost','stop_ignored'))
    a=p.parse_args()
    if (a.supplement_points or a.outside_room_ring) and not (a.bound_ring and a.exhaust_points):
        p.error('supplement/outside ring requires bound ring exhaustion')
    if a.room_case and (a.case!='sequence' or a.multi_point or a.vision_log or a.head_fault or a.head_tf_recovery):
        p.error('room-case requires simple single-point sequence')
    if a.bound_ring and not a.active_ring:p.error('bound-ring requires active-ring')
    if a.turn_only and not a.bound_ring:p.error('turn-only requires bound-ring')
    if a.turn_tf_delay and not a.turn_only:p.error('turn-tf-delay requires turn-only')
    if a.replay_bound and (not a.bound_ring or not a.exhaust_points):p.error('replay-bound requires bound ring exhaustion')
    if a.active_ring and (not a.multi_point or not a.auto_head or a.session_budget_seconds is None
                          or not (a.exhaust_points or a.cancel_at_transition)):
        p.error('active-ring requires multi-point, auto-head, session budget and exhaust-points or cancel-at-transition')
    if (a.slow_navigation or a.budget_stop) and (a.session_budget_seconds is None or a.case!='sequence' or a.multi_point):
        p.error('budget scenarios require single-point sequence and explicit session budget')
    if a.head_tf_recovery and (not a.auto_head or a.case!='sequence' or a.multi_point or a.head_fault or a.vision_log):
        p.error('head-tf-recovery requires unrecorded single-point sequence with auto-head and no head fault')
    if a.head_fault and (not a.auto_head or a.case!='sequence' or a.multi_point):
        p.error('head-fault requires single-point sequence with auto-head')
    assert os.environ.get('ROS_DOMAIN_ID')=='73' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    if a.verify_continuation and a.case!='no_target':p.error('continuation verification requires no_target')
    if a.multi_point and a.case!='sequence':p.error('multi-point test requires sequence')
    if a.cancel_at_transition and not a.multi_point:p.error('cancel-at-transition requires multi-point')
    if (a.exhaust_points or a.expire_during_second) and not a.multi_point:p.error('multi-point required')
    if sum((a.cancel_at_transition,a.exhaust_points,a.expire_during_second))>1:p.error('choose one multi-point scenario')
    recorded=[];recorded_index=0;recorded_hash=None
    if a.vision_log:
        if a.case not in ('sequence','no_target'):p.error('vision-log supports sequence/no_target only')
        data=a.vision_log.read_bytes();recorded_hash=hashlib.sha256(data).hexdigest()
        recorded=[json.loads(line) for line in data.splitlines() if line.strip()]
        recorded=[r for r in recorded if 'sample' in r and r.get('target')=='canned_juice']
        if not recorded:p.error('No canned_juice samples in log')
        if a.case=='sequence' and not any(r['quality'].get('stable') for r in recorded):p.error('No stable samples')
        if a.case=='no_target' and any(any(d['name']=='canned_juice' for d in r['inference']['detections']) for r in recorded):p.error('Expected target-removed recording')
    import rclpy,yaml
    from handyman_msgs.msg import HandymanMsg
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import NavigateToPose,ComputePathToPose
    from rclpy.action import ActionServer,CancelResponse,GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'training/scripts'))
    from verify_live_map import decode
    from rgbd_localization import AUDIT_SHA256
    share=root/'src/handyman_rebuild_ros2';binary=Path('/tmp/handyman-search-nav-install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2')
    decoder=Path('/tmp/handyman-map-snapshot-build/map_snapshot')
    environment=yaml.safe_load((share/'config/environments/layout_a.yaml').read_text())
    task_room='living_room' if a.room_case else 'kitchen'
    search_points=environment['rooms'][task_room]['search_points']
    from search_room_gate import RoomGate
    regions={name:room['region'] for name,room in environment['rooms'].items()}
    def interior(name):
        gate=RoomGate(regions,name);vertices=regions[name]
        for ix in range(1,40):
            for iy in range(1,40):
                p=[min(v[0] for v in vertices)+(max(v[0] for v in vertices)-min(v[0] for v in vertices))*ix/40,
                   min(v[1] for v in vertices)+(max(v[1] for v in vertices)-min(v[1] for v in vertices))*iy/40,1.]
                if gate.classify(p,[0,0,0],[0,0,0,1])['state']=='inside':return p
        raise AssertionError('no_unambiguous_room_fixture')
    target_position=interior('lobby' if a.room_case=='lobby' else task_room)
    if a.room_case=='boundary':target_position=[*regions[task_room][0],1.]
    pose=dict(search_points[0])
    map_msg,_=decode(share/'maps/LayoutA/map.yaml',decoder)
    out=Path(tempfile.mkdtemp(prefix='search-session-runtime-'));print('OUTPUT',out,flush=True)
    fixture_env=dict(os.environ)
    bound_manifest=None
    if a.active_ring:
        from build_active_search_points import build
        generated=build(root)
        layout=next(r for r in generated['reports'] if r['layout']=='LayoutA')
        room=next(r for r in layout['rooms'] if r['room']=='kitchen')
        assert room['status']=='proposed' and len(room['views'])==8
        prefix=out/'isolated-install'
        temporary_share=prefix/'share/handyman_rebuild_ros2'
        shutil.copytree(share/'config',temporary_share/'config')
        shutil.copytree(share/'maps',temporary_share/'maps')
        marker=prefix/'share/ament_index/resource_index/packages/handyman_rebuild_ros2'
        marker.parent.mkdir(parents=True);marker.touch()
        share=temporary_share
        envpath=share/'config/environments/layout_a.yaml'
        environment=yaml.safe_load(envpath.read_text())
        search_points=[{k:v[k] for k in ('x','y','yaw')} for v in room['views']]
        environment['rooms']['kitchen']['search_points']=search_points
        envpath.write_text(yaml.safe_dump(environment))
        pose=dict(search_points[0])
        fixture_env['AMENT_PREFIX_PATH']=str(prefix)+os.pathsep+fixture_env.get('AMENT_PREFIX_PATH','')
        (out/'active-ring-plan.json').write_text(json.dumps(room,indent=2))
        if a.bound_ring:
            from prepare_active_search import snapshot
            bound_manifest=out/'bound-bundle/manifest.json'
            bound=snapshot(root,bound_manifest.parent,'LayoutA','kitchen','apple',generated,a.supplement_points)
            share=Path(bound['package_share'])
            search_points=[p['pose'] for p in bound['request']['points']]
    if a.outside_room_ring:target_position=interior('lobby')
    runtime_directory=out/'runtime';journal=out/'dispatch.sqlite3'
    def persisted():
        if not journal.exists():return []
        with sqlite3.connect(journal) as db:return db.execute('SELECT goal,terminal FROM intents').fetchall()
    rclpy.init();node=rclpy.create_node('session_runtime_acceptance');server_node=rclpy.create_node('session_fake_nav2')
    goals=[];cancelled=[];events=[];results=[];requests=[];statuses=[];stop=threading.Event()
    receipts=[];untracked=[];dropped_acceptances=[];runtime_killed=False
    def accept(goal):
        if a.turn_only:
            index=len(goals)
            from search_turn_policy import same_search_position
            should_turn=index>0 and same_search_position({'pose':search_points[index-1]},{'pose':search_points[index]})
            expected=str(root/'src/handyman_rebuild_ros2/behavior_trees/search_turn.xml') if should_turn else ''
            assert goal.behavior_tree==expected,(index,goal.behavior_tree,expected)
        receipts.append({key for key,status in persisted()})
        if a.case=='reject_once' and len(receipts)==1:return GoalResponse.REJECT
        if a.case=='delayed_accept':time.sleep(1.2)
        return GoalResponse.ACCEPT
    def execute(handle):
        key=bytes(handle.goal_id.uuid).hex();goals.append(key)
        if a.multi_point:
            goalpose=handle.request.pose.pose
            pose.update(x=goalpose.position.x,y=goalpose.position.y,
                        yaw=2*math.atan2(goalpose.orientation.z,goalpose.orientation.w))
        if not any(key in snapshot for snapshot in receipts):untracked.append(key)
        end=time.monotonic()+(0.3 if a.case in ('sequence','no_target','reject_once') else 25.)
        if a.slow_navigation:end=time.monotonic()+65.
        if a.expire_during_second and len(goals)==2:end=time.monotonic()+40.
        while not stop.is_set() and time.monotonic()<end:
            if handle.is_cancel_requested:cancelled.append(key);handle.canceled();return NavigateToPose.Result()
            time.sleep(.01)
        if a.case in ('sequence','no_target','reject_once'):handle.succeed()
        else:handle.abort()
        return NavigateToPose.Result()
    server=ActionServer(server_node,NavigateToPose,'/handyman_test/navigate_to_pose',execute_callback=execute,
        goal_callback=accept,cancel_callback=lambda h:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
    planner=None;planning_calls=[]
    if a.bound_ring:
        def compute_path(handle):
            planning_calls.append(handle.request.use_start)
            result=ComputePathToPose.Result()
            result.path.header.frame_id='map'
            result.path.poses=[copy.deepcopy(handle.request.goal)]
            handle.succeed();return result
        planner=ActionServer(server_node,ComputePathToPose,'/compute_path_to_pose',execute_callback=compute_path,
                             callback_group=ReentrantCallbackGroup())
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(server_node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    moderator=node.create_publisher(HandymanMsg,'/handyman_test/session_to_robot',10)
    budget_events=node.create_publisher(HandymanMsg,'/handyman_test/budget_stop',10)
    replay=node.create_publisher(HandymanMsg,'/handyman_test/session_request',QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    subscriptions=[node.create_subscription(HandymanMsg,'/handyman_test/session_to_moderator',lambda m:events.append(m.message),10),
        node.create_subscription(HandymanMsg,'/handyman_test/session_result',lambda m:results.append(yaml.safe_load(m.detail)),10),
        node.create_subscription(HandymanMsg,'/handyman_test/session_status',lambda m:statuses.append(dict(event=m.message,data=yaml.safe_load(m.detail))),10),
        node.create_subscription(HandymanMsg,'/handyman_test/session_request',lambda m:requests.append(dict(event=m.message,data=yaml.safe_load(m.detail))),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    if a.bound_ring:
        subscriptions.append(node.create_subscription(HandymanMsg,'/handyman_test/session_to_robot',lambda m:None,10))
        def owner_cancel(reason):
            import uuid
            m=HandymanMsg();m.message='search_cancelled'
            m.detail=yaml.safe_dump(dict(schema='handyman-search-request-v1',task_id=bound['task_id'],
                                        cancel_id=uuid.uuid4().hex,reason=reason))
            replay.publish(m)
        def owner_status(msg):
            row=yaml.safe_load(msg.detail)
            if row.get('task_id')==bound['task_id'] and msg.message=='search_stop_requested':
                owner_cancel('search_stop:observation_terminal')
        subscriptions.append(node.create_subscription(HandymanMsg,'/handyman_test/session_status',owner_status,10))
    relays={}
    def relay(msg):
        if msg.message=='owned_goal_accepted':dropped_acceptances.append(msg.detail);return
        row=yaml.safe_load(msg.detail)
        namespace=next(r['namespace'] for r in runtime_rows() if r['event']=='runtime_ready')
        topic=namespace+'/task_'+row['task_id']+'/owned_goals'
        if topic not in relays:relays[topic]=node.create_publisher(HandymanMsg,topic,100)
        relays[topic].publish(msg)
    subscriptions.append(node.create_subscription(HandymanMsg,'/handyman_test/intent_relay',relay,100))
    map_pub=node.create_publisher(OccupancyGrid,'/handyman_test/session_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcaster=TransformBroadcaster(node);vision=node.create_publisher(String,'/handyman_test/session_vision',10)
    head_commands=[];tf_gap_starts={}
    def head_moves():return [c for c in head_commands if c['duration_s']==2.]
    def head_holds():return [c for c in head_commands if c['duration_s']==.2]
    if a.auto_head:
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory
        joint_pub=node.create_publisher(JointState,'/hsrb/joint_states',10)
        def head_command(msg):
            duration=msg.points[0].time_from_start
            head_commands.append(dict(time=time.monotonic(),positions=list(msg.points[0].positions),
                                      duration_s=duration.sec+duration.nanosec/1e9))
        subscriptions.append(node.create_subscription(JointTrajectory,'/hsrb/head_trajectory_controller/command',head_command,10))
    def tick():
        nonlocal recorded_index
        stamp=node.get_clock().now();ns=stamp.nanoseconds
        suppress_tf=False;recovered=False
        if a.turn_tf_delay:
            starts=[r for r in runtime_rows() if r['event']=='observer_started']
            if len(starts)>1 and time.monotonic()-starts[-1]['time']<1.5:
                suppress_tf=True
        if a.head_tf_recovery:
            paths=list(runtime_directory.rglob('observer.jsonl')) if runtime_directory.exists() else []
            if paths:
                latest=max(paths,key=lambda p:p.stat().st_mtime)
                records=[]
                for line in latest.read_text().splitlines():
                    try:records.append(json.loads(line))
                    except ValueError:pass  # concurrent final line may be incomplete
                ready=[r for r in records if r.get('observer_reason')=='head_ready_for_observation']
                if ready and str(latest) not in tf_gap_starts:tf_gap_starts[str(latest)]=time.monotonic()
                start=tf_gap_starts.get(str(latest))
                suppress_tf=start is not None and time.monotonic()-start<.65
                recovered=len(ready)>=2
        if a.auto_head:
            joint=JointState();joint.header.stamp=stamp.to_msg();joint.name=['head_pan_joint','head_tilt_joint']
            joint_ns=ns+a.head_skew_ms*1_000_000
            joint.header.stamp.sec=joint_ns//1_000_000_000
            joint.header.stamp.nanosec=joint_ns%1_000_000_000
            joint.position=[0.,-.25 if head_commands and a.head_fault!='wrong_angle' else 0.]
            if a.head_fault=='stop_ignored' and head_holds():joint.position=[0.,0.]
            if a.head_fault!='missing_feedback' and not (a.head_fault=='stop_feedback_lost' and head_holds()):joint_pub.publish(joint)
        t=TransformStamped();t.header.stamp=stamp.to_msg();t.header.frame_id='odom';t.child_frame_id='base_footprint'
        t.transform.translation.x=pose['x'];t.transform.translation.y=pose['y']
        t.transform.rotation.z=math.sin(pose['yaw']/2);t.transform.rotation.w=math.cos(pose['yaw']/2)
        base=TransformStamped();base.header.stamp=t.header.stamp;base.header.frame_id='map';base.child_frame_id='odom';base.transform.rotation.w=1.
        if not suppress_tf:broadcaster.sendTransform([base,t])
        map_pub.publish(map_msg)
        row=dict(target='apple',geometry_verified=True,audit_sha256=AUDIT_SHA256,rgb_stamp_ns=ns,depth_stamp_ns=ns,
            tf_wait_status='ready',tf_at_depth_stamp={},quality=dict(stable=True,position_m=target_position,frame_id='odom',target='apple',stamp_ns=ns),
            inference=dict(detections=[dict(name='apple')],class_conflicts=[],view_health=dict(schema='handyman-view-health-v1',data_usable=True)))
        if a.case=='no_target' or (a.head_tf_recovery and not recovered):
            row['inference']['detections']=[]
            row['quality']={'stable':False}
        if recorded:
            row=copy.deepcopy(recorded[recorded_index%len(recorded)]);recorded_index+=1
            row['rgb_stamp_ns']=ns;row['depth_stamp_ns']=ns
            if 'stamp_ns' in row['quality']:row['quality']['stamp_ns']=ns
            row['replay_scope']='recorded_detections_and_positions_synthetic_timing_and_navigation'
        if a.multi_point and not a.outside_room_ring and (len(goals)<2 or a.exhaust_points or a.expire_during_second):
            row['inference']['detections']=[];row['quality']={'stable':False}
        msg=String();msg.data=json.dumps(row);vision.publish(msg)
    timer=node.create_timer(.05,tick)
    def send(event,detail=''):
        msg=HandymanMsg();msg.message=event;msg.detail=detail;moderator.publish(msg)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def runtime_rows():
        path=runtime_directory/'events.jsonl'
        return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    def wait(predicate,seconds=15):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return
        raise AssertionError(dict(events=events,goals=goals,statuses=statuses,runtime=runtime_rows()))
    coordinator=None;runtime=None;storage_lock=None
    def kill_child(event):
        child=next(r['pid'] for r in runtime_rows() if r['event']==event)
        stat=Path(f'/proc/{child}/stat').read_text().rsplit(')',1)[1].split()
        assert int(stat[1])==runtime.pid,'only kill test-owned descendant'
        os.kill(child,signal.SIGKILL)
    with (out/'console.log').open('w') as log:
        try:
            coordinator_command=[str(binary/'handyman_coordinator'),'--ros-args','-p','search.publish_requests:=true',
                '-p','navigation.action_name:=/handyman_test/unused_session_nav',
                '-r','/handyman/message/to_robot:=/handyman_test/session_to_robot',
                '-r','/handyman/message/to_moderator:=/handyman_test/session_to_moderator',
                '-r','/handyman/search/request:=/handyman_test/session_request',
                '-r','/handyman/search/execution_status:=/handyman_test/session_status']
            if not a.bound_ring:coordinator=subprocess.Popen(coordinator_command,env=fixture_env,stdout=log,stderr=subprocess.STDOUT)
            worker=binary/'handyman_search_worker';runtime_env=dict(fixture_env)
            if a.case in ('before_send','recovery_unknown','accepted_lost'):
                worker=root/'training/tests/dispatch_worker_fixture.py'
                runtime_env['HANDYMAN_DISPATCH_TEST_MODE']='drop_accepted' if a.case=='accepted_lost' else 'hold_ack'
            runtime_command=[sys.executable,str(root/'training/scripts/search_session_runtime.py'),'--execute',
                '--package-share',str(share),'--worker',str(worker),'--decoder',str(decoder),
                '--journal',str(journal),'--output',str(runtime_directory),'--seconds','80','--observation-seconds',('5' if a.case=='no_target' and not a.verify_continuation else '20'),
                '--action-name','/handyman_test/navigate_to_pose','--map-topic','/handyman_test/session_map',
                '--vision-topic','/handyman_test/session_vision','--request-topic','/handyman_test/session_request',
                '--status-topic','/handyman_test/session_status','--result-topic','/handyman_test/session_result',
                '--private-prefix','/handyman_test/session_runtime']
            if a.verify_continuation:runtime_command+=['--view-seconds','2']
            if a.room_case:runtime_command+=['--view-seconds','3']
            if a.auto_head:runtime_command+=['--head-tilt','-0.25']
            if a.session_budget_seconds is not None:
                runtime_command+=['--session-budget-seconds',str(a.session_budget_seconds),
                                  '--session-event-topic','/handyman_test/budget_stop','--view-seconds','5']
            if a.multi_point:runtime_command+=['--view-seconds','2','--multi-point']
            if a.bound_ring:runtime_command+=['--active-bundle',str(bound_manifest),'--check-current-path']
            if a.turn_only:runtime_command+=['--turn-behavior-tree',str(root/'src/handyman_rebuild_ros2/behavior_trees/search_turn.xml')]
            if a.expire_during_second:runtime_command[runtime_command.index('--observation-seconds')+1]='25'
            runtime=subprocess.Popen(runtime_command,env=runtime_env,stdout=log,stderr=subprocess.STDOUT)
            wait(lambda:runtime_rows() and moderator.get_subscription_count()>0)
            spin(.3)
            if a.session_budget_seconds is not None:
                wait(lambda:budget_events.get_subscription_count()>0)
                for _ in range(3):
                    environment=HandymanMsg();environment.message='Environment';environment.detail='LayoutA'
                    budget_events.publish(environment);spin(.1)
                    heartbeat=HandymanMsg();heartbeat.message='Are_you_ready?'
                    budget_events.publish(heartbeat);spin(.1)
                assert runtime.poll() is None
                assert not any(r['event']=='session_stop_event' for r in runtime_rows())
            rounds=2 if a.case=='sequence' and not a.multi_point and not a.head_fault and a.session_budget_seconds is None else 1
            for i in range(rounds):
                if not a.bound_ring:
                    for attempt in range(30):
                        send('Environment','LayoutA');send('Are_you_ready?');spin(.1)
                        if events.count('I_am_ready')==i+1:break
                    assert events.count('I_am_ready')==i+1
                if a.case=='journal_write_failure':
                    storage_lock=sqlite3.connect(journal);storage_lock.execute('BEGIN IMMEDIATE')
                item='canned juice' if recorded else 'apple'
                if a.bound_ring:
                    msg=HandymanMsg();msg.message='search_requested'
                    msg.detail=yaml.safe_dump(dict(bound['request'],stamp_ns=node.get_clock().now().nanoseconds))
                    replay.publish(msg)
                else:send('Instruction','Go to the '+task_room.replace('_',' ')+', grasp the '+item+' and bring it to the dining table.')
                if a.case=='journal_write_failure':
                    wait(lambda:any(r['event']=='search_cancel_status' and r['data'].get('state')=='cancel_failed' for r in statuses))
                    storage_lock.rollback();storage_lock.close();storage_lock=None
                    assert not goals and not results and not persisted()
                    continue
                if a.case in ('before_send','recovery_unknown'):wait(lambda:bool(persisted()))
                elif a.case=='delayed_accept':wait(lambda:bool(receipts))
                else:wait(lambda:len(goals)==i+1)
                if a.budget_stop:
                    if a.budget_stop=='event':
                        msg=HandymanMsg();msg.message='Mission_complete';budget_events.publish(msg)
                    wait(lambda:runtime.poll() is not None,seconds=25)
                    assert cancelled and not results,(cancelled,results)
                    assert not any(r['event']=='session_retired' for r in runtime_rows())
                    continue
                if a.head_fault:
                    if a.head_fault in ('cancel_during_motion','stop_feedback_lost','stop_ignored'):
                        wait(lambda:bool(head_commands));send('Task_failed')
                        if a.head_fault=='cancel_during_motion':
                            wait(lambda:any(r['event']=='session_retired' for r in runtime_rows()))
                            stops=list(runtime_directory.rglob('head-stop.json'))
                            assert len(stops)==1 and json.loads(stops[0].read_text())['confirmation']=='post_hold_joint_feedback'
                        else:
                            wait(lambda:any(r['event']=='session_fault' for r in runtime_rows()))
                            assert not any(r['event']=='session_retired' for r in runtime_rows())
                            assert not list(runtime_directory.rglob('head-stop.json'))
                        assert len(head_moves())==1 and len(head_holds())==1
                    else:
                        wait(lambda:any(r['event']=='session_fault' for r in runtime_rows()),seconds=15)
                        assert len(head_moves())==(0 if a.head_fault=='missing_feedback' else 1)
                    spin(.3);assert not results and len(goals)==1
                    continue
                if a.expire_during_second:
                    wait(lambda:len(goals)==2)
                    wait(lambda:any(r['event']=='session_fault' for r in runtime_rows()),seconds=35)
                    wait(lambda:goals[1] in cancelled)
                    spin(.5)
                    assert len(goals)==2 and not results
                    assert sum(r['event']=='next_point_prepared' for r in runtime_rows())==1
                    continue
                if a.cancel_at_transition:
                    wait(lambda:any(r['event']=='point_transition_started' for r in runtime_rows()))
                    if a.bound_ring:owner_cancel('fixture_external_cancel')
                    else:send('Task_failed')
                    wait(lambda:any(r['event']=='session_retired' for r in runtime_rows()))
                    spin(.3)
                    assert len(goals)==1 and not results
                    assert not any(r['event']=='next_point_prepared' for r in runtime_rows())
                    continue
                if a.case in ('recovery','recovery_unknown'):
                    old_namespace=next(r['namespace'] for r in runtime_rows() if r['event']=='runtime_ready')
                    runtime.send_signal(signal.SIGSTOP)
                    kill_child('observer_started');kill_child('worker_started')
                    runtime.kill();runtime.wait(timeout=3);runtime_killed=True
                    assert runtime.returncode==-signal.SIGKILL
                    runtime_directory=out/'recovery'
                    runtime_command[runtime_command.index('--output')+1]=str(runtime_directory)
                    runtime=subprocess.Popen(runtime_command,env=runtime_env,stdout=log,stderr=subprocess.STDOUT)
                    wait(lambda:any(r['event']=='recovery_locked' for r in runtime_rows()))
                    assert next(r['namespace'] for r in runtime_rows() if r['event']=='runtime_ready')!=old_namespace
                    if a.case=='recovery':
                        wait(lambda:len(cancelled)==1)
                        wait(lambda:all(status in (4,5,6) for _,status in persisted()))
                    else:
                        spin(2.);assert not goals and any(status is None for _,status in persisted())
                    assert not any(r['event']=='worker_started' for r in runtime_rows()) and not results
                    continue
                if a.case in ('before_send','delayed_accept'):
                    kill_child('worker_started')
                    wait(lambda:any(r['event']=='search_cancel_status' and r['data'].get('state')=='cancel_failed' for r in statuses))
                    if a.case=='before_send':spin(.3);assert not goals
                    else:wait(lambda:len(cancelled)==1)
                    assert not results
                    continue
                if i:
                    old=next(r['data'] for r in requests if r['event']=='search_cancelled')
                    replay_msg=HandymanMsg();replay_msg.message='search_cancelled';replay_msg.detail=yaml.safe_dump(old)
                    replay.publish(replay_msg)
                if a.case in ('sequence','no_target','reject_once'):
                    wait(lambda:len(results)==i+1,seconds=260 if a.active_ring else (80 if a.slow_navigation else (29 if a.exhaust_points else 15)))
                    row=results[-1];assert row['cleanup_verified']
                    if a.case!='no_target' and not a.exhaust_points and a.room_case not in ('lobby','boundary'):
                        assert row['state']=='found'
                        expected=[r['quality']['position_m'] for r in recorded if r['quality'].get('stable')] if recorded else [target_position]
                        assert row['position_m'] in expected
                    else:assert row['state']=='incomplete' and 'position_m' not in row
                    assert row['does_not_exist_authorized'] is False
                    if a.room_case:
                        observations=[json.loads(line) for path in runtime_directory.rglob('observer.jsonl') for line in path.read_text().splitlines()]
                        expected_room_state={'inside':'inside','lobby':'outside_target_room','boundary':'boundary_uncertain'}[a.room_case]
                        assert any(r.get('room_membership',{}).get('state')==expected_room_state for r in observations)
                        if a.room_case=='inside':assert row['room_membership']['target_room']=='living_room'
                    if a.multi_point:
                        expected_goals=len(search_points) if a.exhaust_points else 2
                        assert len(goals)==expected_goals and len(set(goals))==expected_goals
                        history=runtime_rows()
                        assert sum(r['event']=='point_retired' for r in history)==expected_goals-1
                        assert sum(r['event']=='next_point_prepared' for r in history)==expected_goals-1
                        assert row['observer_context']['point_id'].endswith('/'+str(expected_goals-1))
                        assert [r['event'] for r in history].index('point_retired')<[r['event'] for r in history].index('next_point_prepared')
                    if a.verify_continuation:
                        from search_point_sequence import SearchPointSequence
                        context=row['observer_context']
                        policy=SearchPointSequence(row['task_id'],row['target'],context['map_sha256'],[context['point_id'],'next-point-policy-only'])
                        decision=policy.complete_observation(row)
                        assert decision['state']=='awaiting_point' and decision['point_id']=='next-point-policy-only', (row,decision)
                    # The ROS result can arrive before the following JSONL log
                    # write is visible in this process; wait for that evidence.
                    wait(lambda:len([r for r in runtime_rows() if r['event']=='session_retired'])==i+1)
                    retired=[r for r in runtime_rows() if r['event']=='session_retired']
                    assert len(retired)==i+1 and retired[-1]['worker_returncode']==0 and retired[-1]['observer_returncode']==0
                    if a.case=='reject_once':assert sorted(status for _,status in persisted())==[0,4]
                    send('Task_failed');spin(.2)
                elif a.case=='runtime_shutdown':
                    wait(lambda:any(r['event']=='goal_registered' for r in runtime_rows()))
                    runtime.send_signal(signal.SIGINT)
                    wait(lambda:runtime.poll() is not None)
                    assert runtime.returncode==0 and len(cancelled)==1 and not results
                    assert not any(r['event']=='session_retired' for r in runtime_rows())
                elif a.case=='cancel':
                    send('Task_failed');wait(lambda:any(r['event']=='session_retired' for r in runtime_rows()))
                    assert not results and len(cancelled)==1
                else:
                    if a.case=='map_change':map_msg.data[0]=0 if map_msg.data[0]!=0 else 100
                    else:
                        if a.case in ('worker_crash','observer_crash_orphan'):
                            wait(lambda:any(r['event']=='goal_registered' for r in runtime_rows()))
                        if a.case=='observer_crash_orphan':
                            coordinator.terminate();coordinator.wait(timeout=3)
                        event='observer_started' if a.case.startswith('observer_crash') else 'worker_started'
                        if a.case=='accepted_lost':wait(lambda:bool(dropped_acceptances))
                        kill_child(event)
                    wait(lambda:len(cancelled)==1)
                    if a.case=='observer_crash_orphan':
                        wait(lambda:any(r['event']=='session_fault' for r in runtime_rows()))
                        spin(1.)
                        assert not results and not any(r['event']=='session_retired' for r in runtime_rows())
                        continue
                    wait(lambda:any(r['event']=='search_cancel_status' and r['data'].get('state')=='cancel_failed' for r in statuses))
                    send('Task_failed');send('Environment','LayoutA');send('Are_you_ready?');spin(.4)
                    assert events.count('I_am_ready')==1 and not results
                    assert not any(r['event']=='search_cancel_status' and r['data'].get('state')=='cancel_drained' for r in statuses)
            assert not set(events)&{'Does_not_exist','Task_finished','Object_grasped','Give_up'}
            assert not untracked,'Nav2 received a UUID absent from the committed pre-dispatch journal'
            if goals:
                wait(lambda:set(goals).issubset({key for key,status in persisted() if status in (4,5,6)}))
            if a.auto_head and a.case=='sequence' and not a.head_fault and not a.budget_stop:
                observer_logs=list(runtime_directory.rglob('observer.jsonl'))
                assert len(head_moves())==len(observer_logs),(head_commands,observer_logs)
                assert len(head_holds())==len(observer_logs)
                assert len(list(runtime_directory.rglob('head-stop.json')))==len(observer_logs)
                for path,command in zip(sorted(observer_logs,key=lambda p:p.stat().st_mtime),head_moves()):
                    records=[json.loads(line) for line in path.read_text().splitlines()]
                    arrived=next(r for r in records if r.get('arrival_reason')=='arrived_and_settled')
                    ready=next(r for r in records if r.get('observer_reason')=='head_ready_for_observation')
                    assert arrived['monotonic_s']<=command['time']<ready['monotonic_s']
                    assert command['positions']==[0.,-.25]
                    assert records[-1]['state']==('incomplete' if a.active_ring or a.room_case in ('lobby','boundary') else 'found')
                    assert ready['head_stamp_ns']>=arrived['arrival_stamp_ns']
                    if a.head_tf_recovery:
                        reopened=[r for r in records if r.get('observer_reason')=='head_ready_for_observation']
                        assert len(reopened)==2,records
                        assert any(r.get('observer_reason')=='tf_reacquiring' for r in records)
                        assert reopened[1]['head_stamp_ns']>reopened[0]['head_stamp_ns']
            result=dict(passed=True,case=a.case,goals=goals,cancelled=cancelled,events=events,results=results,
                runtime=runtime_rows(),persisted=persisted(),untracked=untracked,runtime_killed=runtime_killed,
                scope=('real runtime/observer/worker/coordinator; recorded RGBD inference/positions, synthetic timestamps/TF/Nav2; no live Unity' if recorded else 'real runtime/observer/worker/coordinator; fake TF/Nav2/RGBD diagnostics; no Unity'),
                vision_log=str(a.vision_log) if a.vision_log else None,vision_log_sha256=recorded_hash)
            if a.verify_continuation:result['continuation_policy_decision']=decision
            result.update(multi_point=a.multi_point,cancel_at_transition=a.cancel_at_transition)
            result.update(exhaust_points=a.exhaust_points,expire_during_second=a.expire_during_second)
            result.update(auto_head=a.auto_head,head_commands=head_commands,head_fault=a.head_fault,
                          head_tf_recovery=a.head_tf_recovery,head_skew_ms=a.head_skew_ms)
            result.update(session_budget_seconds=a.session_budget_seconds,slow_navigation=a.slow_navigation,budget_stop=a.budget_stop)
            result['active_ring']=a.active_ring
            result['bound_ring']=a.bound_ring
            if a.bound_ring:
                result['scope']='real runtime/observer/worker/bundle binding; synthetic private owner, TF/Nav2/RGBD/head; no Unity'
                with sqlite3.connect(journal) as db:
                    claims=db.execute('SELECT task,bundle_sha256 FROM active_requests').fetchall()
                assert len(claims)==1 and claims[0][0]==bound['task_id']
                result['persistent_task_claims']=claims
                assert planning_calls==[False],planning_calls
                result['current_path_preflight_calls']=len(planning_calls)
                if a.replay_bound:
                    goal_count=len(goals)
                    runtime.send_signal(signal.SIGINT);runtime.wait(timeout=8)
                    assert runtime.returncode==0
                    runtime_directory=out/'runtime-replay'
                    runtime_command[runtime_command.index('--output')+1]=str(runtime_directory)
                    runtime=subprocess.Popen(runtime_command,env=runtime_env,stdout=log,stderr=subprocess.STDOUT)
                    wait(lambda:any(r['event']=='runtime_ready' for r in runtime_rows()))
                    msg=HandymanMsg();msg.message='search_requested'
                    msg.detail=yaml.safe_dump(dict(bound['request'],stamp_ns=node.get_clock().now().nanoseconds))
                    replay.publish(msg)
                    wait(lambda:any(r['event']=='request_rejected' and r.get('reason')=='active_request_previously_consumed' for r in runtime_rows()))
                    spin(.3)
                    assert len(goals)==goal_count
                    assert not any(r['event']=='worker_started' for r in runtime_rows())
                    result['replay_after_restart']=dict(passed=True,new_goals=0,events=runtime_rows())
            (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        finally:
            if storage_lock is not None:storage_lock.rollback();storage_lock.close()
            forced=False
            if runtime and runtime.poll() is None:
                runtime.send_signal(signal.SIGINT)
                try:runtime.wait(timeout=8)
                except subprocess.TimeoutExpired:forced=True;runtime.kill();runtime.wait(timeout=3)
            if coordinator and coordinator.poll() is None:send('Mission_complete');spin(.2);coordinator.terminate();coordinator.wait(timeout=3)
            stop.set();executor.shutdown();thread.join(timeout=2);server.destroy()
            if planner:planner.destroy()
            server_node.destroy_node();node.destroy_node();rclpy.try_shutdown()
            remaining=[]
            all_rows=[json.loads(line) for path in out.glob('*/events.jsonl') for line in path.read_text().splitlines()]
            for row in all_rows:
                if row['event'] in ('observer_started','worker_started'):
                    path=Path('/proc')/str(row['pid'])/'cmdline'
                    if path.exists() and any(token in path.read_bytes() for token in (b'search_arrival_ros.py',b'handyman_search_worker')):
                        remaining.append(row['pid'])
            cleanup=dict(runtime_returncode=runtime.poll() if runtime else None,
                forced_runtime_kill=forced,remaining_children=remaining)
            (out/'cleanup.json').write_text(json.dumps(cleanup,indent=2))
            assert not forced and not remaining and (runtime is None or runtime.returncode==0),cleanup

if __name__=='__main__':main()

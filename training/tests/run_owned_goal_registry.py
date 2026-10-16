"""Real executor waypoint/retry registration and crash cancellation; fake Nav2 only."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--case',choices=('waypoint','retry'),required=True)
    parser.add_argument('--supervised',action='store_true')
    parser.add_argument('--unified',action='store_true')
    parser.add_argument('--runtime-worker',action='store_true')
    parser.add_argument('--map-fault',choices=('pending','revoke'))
    parser.add_argument('--live-map',choices=('normal','change','disconnect','silence'))
    parser.add_argument('--lease-fault',choices=('loss','replay','wrong_task','missing','process_crash'))
    parser.add_argument('--cancel-fault',choices=('reject','no_terminal'))
    parser.add_argument('--registration-fault',choices=('drop_registration','drop_ack','wrong_ack','drop_all_ack'))
    args=parser.parse_args()
    if args.runtime_worker and (not args.unified or not args.live_map):parser.error('runtime-worker requires unified and live-map')
    if args.live_map and (not args.unified or args.lease_fault or args.map_fault):parser.error('live-map requires unified without other map/lease faults')
    if args.map_fault and (not args.unified or args.lease_fault):parser.error('map-fault requires unified without lease-fault')
    if args.unified and (not args.supervised or args.case!='waypoint' or args.registration_fault or args.cancel_fault):
        parser.error('unified requires supervised waypoint, optionally lease-fault')
    if args.lease_fault and (not args.supervised or args.case!='waypoint' or args.cancel_fault or args.registration_fault):
        parser.error('lease-fault requires supervised waypoint without other faults')
    if args.cancel_fault and not args.supervised:parser.error('cancel-fault requires supervised')
    if args.registration_fault and (args.case!='waypoint' or args.cancel_fault):
        parser.error('registration-fault uses waypoint without cancel-fault')
    assert os.environ.get('ROS_DOMAIN_ID')=='73' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import rclpy,yaml
    from rclpy.action import ActionServer,CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from handyman_msgs.msg import HandymanMsg
    from rclpy.qos import QoSProfile,DurabilityPolicy
    root=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(root/'training/scripts'))
    from owned_goal_registry import OwnedGoalRegistry,RegisteredGoalsCanceller
    from verify_live_map import bundle
    if args.live_map:
        from verify_live_map import decode
        from search_request_consumer import RequestConsumer,RequestMapGate
        from search_map_responder import SearchMapResponder
        from nav_msgs.msg import OccupancyGrid
        map_msg,map_digest=decode(root/'src/handyman_rebuild_ros2/maps/LayoutA/map.yaml',Path('/tmp/handyman-map-snapshot-build/map_snapshot'))
    env=yaml.safe_load((root/'src/handyman_rebuild_ros2/config/environments/layout_a.yaml').read_text())
    digest=bundle(root/'src/handyman_rebuild_ros2/maps/LayoutA/map.yaml')
    target=env['rooms']['kitchen']['search_points'][0]
    start=env['rooms']['living_room' if args.case=='waypoint' else 'kitchen']['search_points'][0]
    fixture_start=start
    if args.supervised:start=target
    allowed=dict(final_search_point=[target],route_waypoint=[p for r in env['routes'] if r['to']=='kitchen' for p in r['waypoints']])
    task='a'*32;point='LayoutA/kitchen/0'
    registry=OwnedGoalRegistry(task,point,digest,allowed)
    out=Path(tempfile.mkdtemp(prefix='owned-goal-registry-'));print('OUTPUT',out,flush=True)
    rclpy.init();node=rclpy.create_node('owned_goal_test');server_node=rclpy.create_node('owned_goal_fake_nav2')
    manager=None if args.supervised else RegisteredGoalsCanceller(node,registry)
    events=[];requests=[];reports=[]
    worker_reports=[]
    worker_cancel=node.create_publisher(HandymanMsg,'/handyman_test/search_request',
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    moderator=node.create_publisher(HandymanMsg,'/handyman_test/owned_to_robot',10)
    status_pub=node.create_publisher(HandymanMsg,'/handyman_test/owned_status',10)
    subscriptions=[
        node.create_subscription(HandymanMsg,'/handyman_test/search_execution_status',
            lambda m:worker_reports.append(yaml.safe_load(m.detail)),10),
        node.create_subscription(HandymanMsg,'/handyman_test/owned_to_moderator',lambda m:events.append(m.message),10),
        node.create_subscription(HandymanMsg,'/handyman_test/owned_request',lambda m:requests.append(dict(event=m.message,data=yaml.safe_load(m.detail))),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(HandymanMsg,'/handyman_test/owned_status',lambda m:reports.append(dict(event=m.message,data=yaml.safe_load(m.detail))),10)]
    registration_wire=[];ack_wire=[]
    lease_replies=[];lease_fault_active=False
    if args.lease_fault or args.unified:
        lease_forward=node.create_publisher(HandymanMsg,'/handyman_test/lease_reply',10)
        def forward_lease(msg):
            lease_replies.append(msg.detail)
            if args.lease_fault=='missing':return
            if lease_fault_active:
                if args.lease_fault=='loss':return
                row=yaml.safe_load(lease_replies[0] if args.lease_fault=='replay' else msg.detail)
                if args.lease_fault=='wrong_task':row['task_id']='c'*32
                changed=HandymanMsg();changed.message=msg.message;changed.detail=yaml.safe_dump(row)
                lease_forward.publish(changed);return
            lease_forward.publish(msg)
        subscriptions.append(node.create_subscription(HandymanMsg,'/handyman_test/lease_reply_raw',forward_lease,10))
    if args.registration_fault:
        registration_forward=node.create_publisher(HandymanMsg,'/handyman_test/owned_goals',100)
        ack_forward=node.create_publisher(HandymanMsg,'/handyman_test/owned_goal_ack_raw',100)
        def forward_registration(msg):
            registration_wire.append(msg.detail)
            if args.registration_fault=='drop_registration' and len(registration_wire)==1:return
            registration_forward.publish(msg)
        def forward_ack(msg):
            ack_wire.append(msg.detail)
            if args.registration_fault=='drop_all_ack':return
            if args.registration_fault=='drop_ack' and len(ack_wire)==1:return
            if args.registration_fault=='wrong_ack' and len(ack_wire)==1:
                row=yaml.safe_load(msg.detail);row['registration']+='\n# altered acknowledgement'
                wrong=HandymanMsg();wrong.message=msg.message;wrong.detail=yaml.safe_dump(row)
                ack_forward.publish(wrong);return
            ack_forward.publish(msg)
        subscriptions.extend([
            node.create_subscription(HandymanMsg,'/handyman_test/owned_goals_raw',forward_registration,100),
            node.create_subscription(HandymanMsg,'/handyman_test/owned_goal_ack',forward_ack,100)])
    def send(event,detail=''):
        msg=HandymanMsg();msg.message=event;msg.detail=detail;moderator.publish(msg)
    goals=[];active=set();cancel_requests=[];cancelled=[];stop=threading.Event()
    def execute(handle):
        key=bytes(handle.goal_id.uuid).hex();goals.append(key);active.add(key)
        if args.case=='retry' and len(goals)==1:
            active.remove(key);handle.abort();return NavigateToPose.Result()
        end=time.monotonic()+12
        while not stop.is_set() and time.monotonic()<end:
            if handle.is_cancel_requested and args.cancel_fault!='no_terminal':
                active.remove(key);cancelled.append(key);handle.canceled();return NavigateToPose.Result()
            time.sleep(.01)
        active.discard(key);handle.abort();return NavigateToPose.Result()
    def cancel(handle):
        cancel_requests.append(bytes(handle.goal_id.uuid).hex())
        return CancelResponse.REJECT if args.cancel_fault=='reject' else CancelResponse.ACCEPT
    server=ActionServer(server_node,NavigateToPose,'/handyman_test/navigate_to_pose',
        execute_callback=execute,cancel_callback=cancel,callback_group=ReentrantCallbackGroup())
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(server_node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    broadcaster=TransformBroadcaster(node)
    def tick():
        tf=TransformStamped();tf.header.stamp=node.get_clock().now().to_msg()
        tf.header.frame_id='map';tf.child_frame_id='base_footprint'
        tf.transform.translation.x=float(start['x']);tf.transform.translation.y=float(start['y']);tf.transform.rotation.w=1.
        broadcaster.sendTransform(tf)
    timer=node.create_timer(.05,tick)
    def wait(predicate,seconds=6):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return
        raise AssertionError(dict(goals=goals,events=events,requests=requests,
            diagnostics=manager.diagnostics() if manager else None))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    coordinator=None;child=None;supervisor=None;supervisor_process=None
    with (out/'console.log').open('w') as log:
        try:
            if args.supervised:
                coordinator=subprocess.Popen([
                    '/tmp/handyman-search-nav-install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2/handyman_coordinator',
                    '--ros-args','-p','search.publish_requests:=true',
                    '-p','navigation.action_name:=/handyman_test/unused_owned_room_nav',
                    '-r','/handyman/message/to_robot:=/handyman_test/owned_to_robot',
                    '-r','/handyman/message/to_moderator:=/handyman_test/owned_to_moderator',
                    '-r','/handyman/search/request:=/handyman_test/owned_request',
                    '-r','/handyman/search/execution_status:=/handyman_test/owned_status'],
                    stdout=log,stderr=subprocess.STDOUT)
                wait(lambda:moderator.get_subscription_count()>0);spin(.3)
                for i in range(20):
                    send('Environment','LayoutA');send('Are_you_ready?');spin(.1)
                    if 'I_am_ready' in events:break
                assert events.count('I_am_ready')==1
                send('Instruction','Go to the kitchen, grasp the apple and bring it to the dining table.')
                wait(lambda:any(r['event']=='search_requested' for r in requests))
                request=next(r['data'] for r in requests if r['event']=='search_requested')
                assert request['room']=='kitchen' and request['points'][0]['pose']==target
                assert request['points'][0]['id']==point and request['layout']=='LayoutA'
                task=request['task_id']
                if args.live_map:
                    consumer=RequestConsumer(root/'src/handyman_rebuild_ros2')
                    consumer.receive('search_requested',request,node.get_clock().now().nanoseconds,time.monotonic())
                    import copy
                    map_gate=RequestMapGate(consumer);map_gate.install(task,copy.deepcopy(map_msg),map_digest)
                    map_pub=node.create_publisher(OccupancyGrid,'/handyman_test/guard_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
                    def map_publishers():
                        return sorted(bytes(p.endpoint_gid).hex() for p in node.get_publishers_info_by_topic('/handyman_test/guard_map'))
                    def on_map(msg):
                        known=map_publishers()
                        map_gate.receive_map(msg,known[0] if len(known)==1 else None,known)
                    subscriptions.append(node.create_subscription(OccupancyGrid,'/handyman_test/guard_map',on_map,
                        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
                    map_timer=node.create_timer(.1,lambda:map_pub.publish(map_msg))
                    map_responder=SearchMapResponder(node,consumer,map_gate,map_publishers,
                        request_topic='/handyman_test/map_check_request',reply_topic='/handyman_test/map_check_reply')
                registry=OwnedGoalRegistry(task,point,digest,allowed)
                manager=RegisteredGoalsCanceller(node,registry)
                # Synthetic test setup only: place fixture at route start after
                # coordinator request emission; no physical navigation is implied.
                start=fixture_start;spin(.2)
            child_env=dict(os.environ,HANDYMAN_TEST_WORKER_STATUS='1')
            if args.unified:child_env['HANDYMAN_TEST_UNIFIED_GUARD']='1'
            if args.live_map:child_env['HANDYMAN_TEST_LIVE_MAP']='1'
            if args.map_fault=='pending':child_env['HANDYMAN_TEST_MAP_PENDING']='1'
            if args.lease_fault:child_env['HANDYMAN_TEST_SUPERVISOR_LEASE']='1'
            if args.registration_fault:child_env['HANDYMAN_TEST_REG_ACK']='1'
            command=['/tmp/handyman-drain-fixture-build/search_binding_fixture',
                str(root/'src/handyman_rebuild_ros2/config/environments.yaml'),'LayoutA',task,point,digest]
            if args.runtime_worker:
                command=['/tmp/handyman-search-nav-install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2/handyman_search_worker','--ros-args']
                params={'execution.enabled':'true','catalog':str(root/'src/handyman_rebuild_ros2/config/environments.yaml'),
                    'environment':'LayoutA','room':'kitchen','task_id':task,'point_id':point,'point_index':'0',
                    'map_sha256':digest,'navigation.action_name':'/handyman_test/navigate_to_pose'}
                for key,value in params.items():command+=['-p',key+':='+value]
                remaps={'map_check/request':'map_check_request','map_check/reply':'map_check_reply',
                    'lease/request':'lease_request','lease/reply':'lease_reply','owned_goals':'owned_goals',
                    'owned_goal_ack':'owned_goal_ack','request':'owned_request','execution_status':'owned_status',
                    'binding':'search_binding','retire':'retire'}
                for source,target in remaps.items():command+=['-r','/handyman/search/'+source+':=/handyman_test/'+target]
            child=subprocess.Popen(command,
                stdout=log,stderr=subprocess.STDOUT,env=child_env)
            if args.supervised and args.lease_fault=='process_crash':
                supervisor_process=subprocess.Popen([sys.executable,
                    str(root/'training/tests/lease_supervisor_fixture.py'),task,str(child.pid)],
                    stdout=log,stderr=subprocess.STDOUT)
            elif args.supervised:
                from search_process_supervisor import SearchProcessSupervisor
                from search_process_watchdog import SearchProcessWatchdog
                supervisor=SearchProcessSupervisor(node,SearchProcessWatchdog(task,child,15.),
                    request_topic='/handyman_test/owned_request',status_topic='/handyman_test/owned_status',
                    goal_canceller=manager,lease_test=bool(args.lease_fault or args.unified),
                    retire_topic='/handyman_test/retire' if args.runtime_worker else None,
                    retirement_check=lambda:bool(registry.entries) and all(k in manager.results for k in registry.entries))
            if args.live_map and args.live_map!='normal':
                wait(lambda:len(goals)==1 and len(registry.entries)==1)
                assert consumer.active['live_map_verified']
                wait(lambda:'Registration acknowledged:' in (out/'console.log').read_text())
                if args.live_map=='change':map_msg.data[0]=0 if map_msg.data[0]!=0 else 100
                elif args.live_map=='disconnect':map_timer.cancel();node.destroy_publisher(map_pub)
                else:node.destroy_subscription(map_responder.subscription)
                wait(lambda:any(r['event']=='search_cancelled' for r in requests))
                notice=next(r['data'] for r in requests if r['event']=='search_cancelled')
                assert notice['reason']=='worker_fault:map_verification_revoked'
                wait(lambda:cancelled==goals and not active)
                assert cancel_requests==goals
                wait(lambda:any(r['data'].get('state')=='cancel_failed' for r in reports))
                assert not any(r['data'].get('state')=='cancel_drained' for r in reports)
                send('Task_failed');send('Environment','LayoutA');send('Are_you_ready?');spin(.3)
                assert events.count('I_am_ready')==1
                result=dict(passed=True,runtime_worker=args.runtime_worker,live_map=args.live_map,goals=goals,cancel_requests=cancel_requests,
                    reports=reports,scope='real RequestMapGate -> ROS evidence -> guard; published decoded map; fake Nav2/TF; no Unity')
                (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
                send('Mission_complete');coordinator.wait(timeout=3);return
            if args.map_fault:
                if args.map_fault=='pending':
                    spin(2.8);assert not goals and child.poll() is None
                    assert not any(r['event']=='search_worker_fault' for r in reports)
                else:
                    wait(lambda:len(goals)==1 and len(registry.entries)==1)
                    wait(lambda:'Registration acknowledged:' in (out/'console.log').read_text())
                    control=node.create_publisher(HandymanMsg,'/handyman_test/map_control',10)
                    wait(lambda:control.get_subscription_count()>0)
                    msg=HandymanMsg();msg.message='revoke';msg.detail=task;control.publish(msg)
                    wait(lambda:any(r['event']=='search_cancelled' for r in requests))
                    notice=next(r['data'] for r in requests if r['event']=='search_cancelled')
                    assert notice['reason']=='worker_fault:map_verification_revoked'
                    wait(lambda:cancelled==goals and not active)
                    assert cancel_requests==goals
                    wait(lambda:any(r['data'].get('state')=='cancel_failed' for r in reports))
                    assert not any(r['data'].get('state')=='cancel_drained' for r in reports)
                    send('Task_failed');send('Environment','LayoutA');send('Are_you_ready?');spin(.3)
                    assert events.count('I_am_ready')==1
                result=dict(passed=True,map_fault=args.map_fault,goals=goals,cancel_requests=cancel_requests,
                    reports=reports,scope='injected map evidence, real guard; no actual map verification or Unity')
                (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
                send('Mission_complete');coordinator.wait(timeout=3);return
            if args.unified and not args.lease_fault:
                wait(lambda:len(goals)==1 and len(registry.entries)==1)
                wait(lambda:'Registration acknowledged:' in (out/'console.log').read_text())
                spin(.3);assert active=={goals[0]} and not cancel_requests
                if args.runtime_worker:
                    premature=node.create_publisher(HandymanMsg,'/handyman_test/retire',10)
                    wait(lambda:premature.get_subscription_count()>0)
                    msg=HandymanMsg();msg.message='search_retire'
                    msg.detail=yaml.safe_dump(dict(schema='handyman-search-retire-v1',task_id=task,cancel_id='b'*32))
                    premature.publish(msg);spin(.2);assert child.poll() is None
                send('Task_failed')
                wait(lambda:any(r['event']=='search_cancel_status' and r['data']['state']=='cancel_drained' for r in reports))
                spin(1.2)
                assert cancelled==goals and cancel_requests==goals and not active
                assert not any(r['event']=='search_worker_fault' for r in reports)
                if args.runtime_worker:
                    wait(lambda:supervisor.watchdog.retired)
                    assert child.poll()==0 and supervisor.watchdog.fault is None
                    assert 'Confirmed search retirement' in (out/'console.log').read_text()
                send('Environment','LayoutA');send('Are_you_ready?');spin(.3)
                assert events.count('I_am_ready')==2 and len(goals)==1
                result=dict(passed=True,runtime_worker=args.runtime_worker,retired=supervisor.watchdog.retired,
                    worker_returncode=child.poll(),unified=True,live_map=args.live_map,goals=goals,cancel_requests=cancel_requests,
                    reports=reports,events=events,scope='all three protections enabled; fake TF/Nav2; no production authorization')
                (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
                send('Mission_complete');coordinator.wait(timeout=3);return
            if args.lease_fault:
                if args.lease_fault!='missing':
                    wait(lambda:len(goals)==1 and len(registry.entries)==1)
                    spin(.3);assert active=={goals[0]} and not cancel_requests
                    if args.lease_fault=='process_crash':
                        assert supervisor_process.poll() is None and lease_replies
                        supervisor_process.kill();supervisor_process.wait(timeout=3)
                        assert supervisor_process.returncode==-9
                    else:lease_fault_active=True
                wait(lambda:any(r['event']=='search_cancelled' for r in requests))
                notice=next(r['data'] for r in requests if r['event']=='search_cancelled')
                assert notice['reason']=='worker_fault:supervisor_lease_timeout'
                assert child.poll() is None
                if args.lease_fault=='missing':assert not goals and not cancel_requests
                else:
                    wait(lambda:cancelled==goals and not active)
                    assert cancel_requests==goals
                # Restore delivery and inject the latest legitimate reply. A
                # post-deadline response must never unseal the executor.
                lease_fault_active=False
                assert lease_replies
                late=HandymanMsg();late.message='supervisor_lease_reply';late.detail=lease_replies[-1]
                lease_forward.publish(late);spin(1.2)
                assert len(goals)==(0 if args.lease_fault=='missing' else 1)
                send('Task_failed');send('Environment','LayoutA');send('Are_you_ready?');spin(.3)
                assert events.count('I_am_ready')==1
                assert sum(r['event']=='search_cancelled' for r in requests)==1
                assert not set(events)&{'Task_finished','Does_not_exist','Give_up'}
                result=dict(passed=True,unified=args.unified,lease_fault=args.lease_fault,goals=goals,cancel_requests=cancel_requests,
                    cancelled=cancelled,requests=requests,reports=reports,events=events,
                    supervisor_returncode=supervisor_process.returncode if supervisor_process else None,
                    worker_alive=child.poll() is None,
                    scope=('isolated separate supervisor SIGKILL; parent owns worker; fake Nav2/TF; no Unity'
                        if args.lease_fault=='process_crash' else
                        'isolated real executor/coordinator and supervisor adapter; dropped/corrupted replies, not OS process crash; no Unity'))
                (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
                send('Mission_complete');coordinator.wait(timeout=3)
                return
            count=1 if args.case=='waypoint' else 2
            wait(lambda:len(goals)==count and len(registry.entries)==count)
            if args.case=='retry':wait(lambda:manager.results.get(goals[0])==6)
            assert registry.entries[goals[-1]]['role']==('route_waypoint' if args.case=='waypoint' else 'final_search_point')
            assert active=={goals[-1]}
            if args.registration_fault:
                terminal_log=('Registration timed out: ' if args.registration_fault=='drop_all_ack'
                    else 'Registration acknowledged: ')+goals[-1]
                wait(lambda:terminal_log in (out/'console.log').read_text())
                assert len(registration_wire)>=2 and len(set(registration_wire))==1
                if args.registration_fault in ('drop_ack','wrong_ack','drop_all_ack'):assert len(ack_wire)>=2
                if args.registration_fault=='drop_all_ack':
                    wait(lambda:manager.results.get(goals[-1])==5 and not active)
                    late=HandymanMsg();late.message='owned_goal_registered';late.detail=ack_wire[0]
                    ack_forward.publish(late);spin(.2)
                    assert 'Registration acknowledged:' not in (out/'console.log').read_text()
                else:
                    count_after_ack=len(registration_wire);spin(.35)
                    assert len(registration_wire)<=count_after_ack+1, 'retransmission did not stop after ack'
                    assert 'Registration timed out:' not in (out/'console.log').read_text()
            if args.supervised and args.registration_fault!='drop_all_ack':
                wrong=HandymanMsg();wrong.message='search_worker_fault'
                wrong.detail=yaml.safe_dump(dict(schema='handyman-search-worker-fault-v1',task_id='c'*32,
                    reason='worker_exited_without_retirement',returncode=-9))
                status_pub.publish(wrong);spin(.2)
                assert not cancel_requests and not any(r['event']=='search_cancelled' for r in requests)
            if args.supervised and args.registration_fault=='drop_all_ack':
                wait(lambda:any(r['event']=='search_cancelled' for r in requests))
                assert child.poll() is None, 'registration fault must be reported before worker exit'
                notice=next(r['data'] for r in requests if r['event']=='search_cancelled')
                assert notice['reason']=='worker_fault:goal_registration_timeout'
                direct=HandymanMsg();direct.message='search_cancelled';direct.detail=yaml.safe_dump(notice)
                worker_cancel.publish(direct)
                wait(lambda:any(r.get('state')=='cancel_failed' for r in worker_reports))
                assert not any(r.get('state')=='cancel_drained' for r in worker_reports)
                send('Task_failed');send('Environment','LayoutA');send('Are_you_ready?');spin(.3)
                assert events.count('I_am_ready')==1 and len(goals)==count
            child.kill();child.wait(timeout=3);assert child.returncode!=0
            if args.supervised:
                wait(lambda:any(r['event']=='search_cancelled' for r in requests))
                notice=next(r['data'] for r in requests if r['event']=='search_cancelled')
                expected_reason=('goal_registration_timeout' if args.registration_fault=='drop_all_ack'
                    else 'worker_exited_without_retirement')
                assert notice['task_id']==task and notice['reason']=='worker_fault:'+expected_reason
                wait(lambda:manager.cancel is not None)
                assert manager.cancel['cancel_id']==notice['cancel_id']
            else:
                notice=dict(schema='handyman-search-request-v1',task_id=task,cancel_id='b'*32)
                assert not manager.request_cancel(dict(notice,task_id='c'*32))
                assert cancel_requests==([goals[-1]] if args.registration_fault=='drop_all_ack' else [])
                assert manager.request_cancel(notice)
            if args.registration_fault=='drop_all_ack':
                wait(lambda:goals[-1] in manager.jobs)
                assert manager.jobs[goals[-1]] is None
            else:
                wait(lambda:goals[-1] in manager.jobs and manager.jobs[goals[-1]].finished)
            expected=('rejected_or_uuid_mismatch' if args.cancel_fault=='reject' else
                'timeout' if args.cancel_fault else 'goal_cancel_confirmed')
            if args.registration_fault!='drop_all_ack':
                assert manager.jobs[goals[-1]].events[-1]['state']==expected
            assert cancel_requests==[goals[-1]]
            if args.cancel_fault:
                assert not cancelled and active=={goals[-1]}
            else:
                assert cancelled==[goals[-1]] and not active
            if args.case=='retry':
                assert manager.jobs[goals[0]] is None
                assert any(e['state']=='already_terminal' and e['goal_id']==goals[0] for e in manager.events)
            assert not any(e['state']=='registration_rejected' for e in manager.events)
            if args.supervised:
                wait(lambda:any(r['event']=='search_cancel_status' and r['data']['state']=='cancel_failed' for r in reports))
                assert all(r['data']['task_id']==task and r['data']['cancel_id']==notice['cancel_id']
                    for r in reports if r['event']=='search_cancel_status')
                # After internal cancellation, reset protocol and try another task.
                send('Task_failed');send('Environment','LayoutA');send('Are_you_ready?')
                send('Instruction','Go to the kitchen, grasp the apple and bring it to the dining table.');spin(.3)
                assert events.count('I_am_ready')==1 and len(goals)==count
                assert sum(r['event']=='search_requested' for r in requests)==1
                assert sum(r['event']=='search_cancelled' for r in requests)==1
                assert not set(events)&{'Task_finished','Object_grasped','Does_not_exist','Give_up'}
                assert not any(r['data'].get('state')=='cancel_drained' for r in reports)
                assert 'Ready handshake blocked' in (out/'console.log').read_text()
                send('Mission_complete');coordinator.wait(timeout=3);assert coordinator.returncode==0
            result=dict(passed=True,case=args.case,goals=goals,cancel_requests=cancel_requests,
                cancelled=cancelled,diagnostics=manager.diagnostics(),supervised=args.supervised,
                cancel_fault=args.cancel_fault,events=events,requests=requests,reports=reports,
                registration_fault=args.registration_fault,registration_wire=registration_wire,ack_wire=ack_wire,
                worker_reports=worker_reports,
                scope=('real coordinator + supervisor + registered-goal cancellation; fake TF/Nav2; no Unity'
                    if args.supervised else 'real executor; fake Nav2; manually supplied cancellation context; no coordinator or Unity'))
            (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        finally:
            stop.set()
            if supervisor_process is not None and supervisor_process.poll() is None:
                supervisor_process.terminate();supervisor_process.wait(timeout=3)
            if child is not None and child.poll() is None:child.terminate();child.wait(timeout=3)
            if coordinator is not None and coordinator.poll() is None:coordinator.terminate();coordinator.wait(timeout=3)
            executor.shutdown();thread.join(timeout=2);server.destroy();server_node.destroy_node();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()

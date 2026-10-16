"""Isolated coordinator request smoke test; no real action server or robot."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import sys
import argparse
import math
import threading


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--map-change',action='store_true')
    parser.add_argument('--execute-search',action='store_true',help='Isolated fake-Nav2 chain only')
    parser.add_argument('--stop-during',choices=('cancel','map','late_accept'))
    parser.add_argument('--cancel-fault',choices=('reject','no_terminal','no_response'))
    parser.add_argument('--worker-status', action='store_true')
    parser.add_argument('--process-fault', choices=('crash','deadline'))
    parser.add_argument('--independent-cancel', action='store_true')
    args=parser.parse_args()
    if args.execute_search and args.map_change:parser.error('Run these scenarios separately')
    if args.stop_during and not args.execute_search:parser.error('stop-during requires execute-search')
    if args.cancel_fault and args.stop_during!='cancel':parser.error('cancel-fault requires stop-during cancel')
    if args.worker_status and (not args.execute_search or args.stop_during=='map'):
        parser.error('worker-status requires execute-search without stop-during map')
    if args.process_fault and (not args.worker_status or args.stop_during!='cancel' or (args.cancel_fault and not args.independent_cancel)):
        parser.error('process-fault requires worker-status, stop-during cancel, and no cancel-fault')
    if args.independent_cancel and args.process_fault!='crash':
        parser.error('independent-cancel requires process-fault crash')
    assert os.environ.get('ROS_DOMAIN_ID')=='73' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import yaml
    import rclpy
    from handyman_msgs.msg import HandymanMsg
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer,CancelResponse,GoalResponse
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from std_msgs.msg import String
    root=Path(__file__).resolve().parents[2]
    env=yaml.safe_load((root/'src/handyman_rebuild_ros2/config/environments/layout_a.yaml').read_text())
    pose=env['rooms']['kitchen']['search_points'][0]
    sys.path.insert(0,str(root/'training/scripts'))
    from verify_live_map import decode
    map_msg,_=decode(root/'src/handyman_rebuild_ros2/maps/LayoutA/map.yaml',Path('/tmp/handyman-map-snapshot-build/map_snapshot'))
    out=Path(tempfile.mkdtemp(prefix='coordinator-search-request-'))
    print('OUTPUT',out,flush=True)
    rclpy.init();node=rclpy.create_node('coordinator_search_request_test')
    events=[];requests=[]
    pub=node.create_publisher(HandymanMsg,'/handyman_test/to_robot',10)
    sub=node.create_subscription(HandymanMsg,'/handyman_test/to_moderator',lambda m:events.append(m.message),10)
    req=node.create_subscription(HandymanMsg,'/handyman_test/search_request',
        lambda m:requests.append(dict(event=m.message,data=yaml.safe_load(m.detail))),
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcaster=TransformBroadcaster(node)
    map_pub=node.create_publisher(OccupancyGrid,'/handyman_test/request_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    map_timer=node.create_timer(.1,lambda:map_pub.publish(map_msg))
    search_processes=[];goal_ids=[];observer_rows=[]
    cancelled_ids=[];cancel_requests=[]
    observer_cancel_latency=None
    cancellation_reports=[]
    worker_reports=[]
    worker_fault_notices=[]
    supervisor=None;watchdog=None;inflight_evidence=None;independent=None
    unrelated_active=threading.Event();unrelated_release=threading.Event();unrelated_id=None
    active_goal_ids=set()
    def worker_status_received(m):
        row=yaml.safe_load(m.detail)
        (worker_reports if m.message=='search_cancel_status' else worker_fault_notices).append(row)
    worker_sub=node.create_subscription(HandymanMsg,'/handyman_test/search_execution_status',
        worker_status_received,10)
    acceptance_pending=threading.Event();release_acceptance=threading.Event();timeline=[];cancel_acks=[]
    def ack_received(msg):
        if msg.message=='executor_cancel_called':cancel_acks.append(msg.detail)
        else:cancellation_reports.append(dict(state=msg.message,goal_id=msg.detail))
    ack_sub=node.create_subscription(HandymanMsg,'/handyman_test/search_cancel_ack',ack_received,10)
    fake_node=None;fake_executor=None;fake_thread=None;fake_server=None
    if args.execute_search:
        fake_node=rclpy.create_node('fake_nav2_for_full_search_chain')
        def execute(handle):
            if args.independent_cancel and handle.request.pose.pose.position.x == -100.:
                unrelated_active.set()
                deadline=time.monotonic()+30
                while not unrelated_release.is_set() and time.monotonic()<deadline:
                    if handle.is_cancel_requested:
                        unrelated_active.clear();handle.canceled();return NavigateToPose.Result()
                    time.sleep(.02)
                unrelated_active.clear();handle.abort();return NavigateToPose.Result()
            goal_ids.append(bytes(handle.goal_id.uuid).hex())
            active_goal_ids.add(bytes(handle.goal_id.uuid).hex())
            requested=handle.request.pose.pose
            assert abs(requested.position.x-pose['x'])<1e-9 and abs(requested.position.y-pose['y'])<1e-9
            if args.stop_during:
                end=time.monotonic()+8
                while time.monotonic()<end:
                    if handle.is_cancel_requested and args.cancel_fault!='no_terminal':
                        cancelled_ids.append(bytes(handle.goal_id.uuid).hex())
                        active_goal_ids.discard(bytes(handle.goal_id.uuid).hex())
                        handle.canceled();return NavigateToPose.Result()
                    time.sleep(.02)
                active_goal_ids.discard(bytes(handle.goal_id.uuid).hex())
                handle.abort();return NavigateToPose.Result()
            time.sleep(.5);active_goal_ids.discard(bytes(handle.goal_id.uuid).hex())
            handle.succeed();return NavigateToPose.Result()
        def cancel_goal(handle):
            timeline.append(dict(event='server_cancel_received',time=time.monotonic()))
            cancel_requests.append(bytes(handle.goal_id.uuid).hex())
            if args.cancel_fault=='no_response':time.sleep(2.5)
            return CancelResponse.REJECT if args.cancel_fault=='reject' else CancelResponse.ACCEPT
        def accept_goal(request):
            if args.stop_during=='late_accept':
                acceptance_pending.set()
                if not release_acceptance.wait(timeout=5):return GoalResponse.REJECT
                timeline.append(dict(event='server_acceptance_released',time=time.monotonic()))
            return GoalResponse.ACCEPT
        fake_server=ActionServer(fake_node,NavigateToPose,'/handyman_test/navigate_to_pose',
            execute_callback=execute,cancel_callback=cancel_goal,goal_callback=accept_goal,
            callback_group=ReentrantCallbackGroup())
        fake_executor=MultiThreadedExecutor(num_threads=3);fake_executor.add_node(fake_node)
        fake_thread=threading.Thread(target=fake_executor.spin,daemon=True);fake_thread.start()
    def tf_tick():
        tf=TransformStamped();tf.header.stamp=node.get_clock().now().to_msg()
        tf.header.frame_id='odom';tf.child_frame_id='base_footprint'
        tf.transform.translation.x=float(pose['x']);tf.transform.translation.y=float(pose['y'])
        tf.transform.rotation.z=math.sin(pose['yaw']/2);tf.transform.rotation.w=math.cos(pose['yaw']/2)
        root_tf=TransformStamped();root_tf.header.stamp=tf.header.stamp
        root_tf.header.frame_id='map';root_tf.child_frame_id='odom';root_tf.transform.rotation.w=1.
        broadcaster.sendTransform([root_tf,tf])
    timer=node.create_timer(.05,tf_tick)
    def send(event,detail=''):
        m=HandymanMsg();m.message=event;m.detail=detail;pub.publish(m)
    def wait(predicate,seconds=5):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return
        raise AssertionError(dict(events=events,requests=requests))
    with (out/'console.log').open('w') as log:
        consumer=subprocess.Popen([sys.executable,str(root/'training/scripts/search_request_consumer.py'),
            '--topic','/handyman_test/search_request','--seconds','15' if args.execute_search else '8','--output',str(out/'consumer.jsonl'),
            '--decoder','/tmp/handyman-map-snapshot-build/map_snapshot','--map-topic','/handyman_test/request_map'],
            stdout=log,stderr=subprocess.STDOUT)
        process=subprocess.Popen(['/tmp/handyman-search-nav-install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2/handyman_coordinator',
            '--ros-args','-p','search.publish_requests:=true',
            '-r','/handyman/message/to_robot:=/handyman_test/to_robot',
            '-r','/handyman/message/to_moderator:=/handyman_test/to_moderator',
            '-r','/handyman/search/request:=/handyman_test/search_request',
            '-r','/handyman/search/execution_status:=/handyman_test/search_execution_status',
            '-p','navigation.action_name:=/handyman_test/unused_nav'],stdout=log,stderr=subprocess.STDOUT)
        try:
            wait(lambda:pub.get_subscription_count()>0)
            wait(lambda:node.count_subscribers('/handyman_test/search_request')>=2)
            # Repeated handshake mirrors protocol startup discovery, not instruction replay.
            for i in range(30):
                send('Environment','LayoutA');send('Are_you_ready?')
                for j in range(4):rclpy.spin_once(node,timeout_sec=.05)
                if 'I_am_ready' in events:break
            assert 'I_am_ready' in events
            for i in range(15):rclpy.spin_once(node,timeout_sec=.05)
            send('Instruction','Go to the kitchen, grasp the apple and bring it to the dining table.')
            wait(lambda:any(r['event']=='search_requested' for r in requests))
            request=next(r['data'] for r in requests if r['event']=='search_requested')
            assert request['target']=='apple' and request['room']=='kitchen'
            assert request['points'][0]['pose']==pose
            assert request['requires_map_verification'] and not request['actionable']
            assert 'Room_reached' in events
            def consumer_rows():
                path=out/'consumer.jsonl'
                return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
            wait(lambda:any(r['state']=='prepared_readonly' for r in consumer_rows()))
            wait(lambda:any(r['state']=='map_content_verified' for r in consumer_rows()))
            if args.execute_search:
                from verify_live_map import bundle
                from rgbd_localization import AUDIT_SHA256
                verified=consumer_rows()[-1]
                assert verified['state']=='map_content_verified' and verified['task_id']==request['task_id']
                assert bundle(Path(verified['map_path']))==verified['map_bundle_sha256']
                chosen=verified['points'][0]
                assert chosen['id']==request['points'][0]['id'] and chosen['pose']==pose
                observer=subprocess.Popen([sys.executable,str(root/'training/scripts/search_arrival_ros.py'),
                    '--binding-topic','/handyman_test/search_binding','--task-id',request['task_id'],
                    '--point-id',chosen['id'],'--map-sha256',verified['map_bundle_sha256'],
                    '--x',str(pose['x']),'--y',str(pose['y']),'--yaw',str(pose['yaw']),
                    '--target',request['target'],'--motion-source','tf','--seconds','10',
                    '--cancel-topic','/handyman_test/search_request',
                    '--action-status-topic','/handyman_test/navigate_to_pose/_action/status',
                    '--output',str(out/'observer.jsonl')],stdout=log,stderr=subprocess.STDOUT)
                search_processes.append(observer)
                wait(lambda:node.count_subscribers('/handyman_test/search_binding')==1)
                if args.independent_cancel:
                    from independent_goal_cancel import IndependentGoalCanceller
                    from types import SimpleNamespace
                    independent=IndependentGoalCanceller(node,SimpleNamespace(task_id=request['task_id'],
                        point_id=chosen['id'],map_sha256=verified['map_bundle_sha256'],**pose),
                        binding_topic='/handyman_test/search_binding')
                # Test-only authorization: the C++ fixture refuses all domains except 73.
                fixture=subprocess.Popen([('/tmp/handyman-drain-fixture-build/search_binding_fixture' if args.worker_status
                    else '/tmp/handyman-binding-fixture-build/search_binding_fixture'),
                    str(Path(verified['environment_path']).parent.parent/'environments.yaml'),
                    request['environment'],request['task_id'],chosen['id'],verified['map_bundle_sha256']],
                    stdout=log,stderr=subprocess.STDOUT,
                    env=dict(os.environ, HANDYMAN_TEST_WORKER_STATUS='1') if args.worker_status else os.environ)
                search_processes.append(fixture)
                if args.stop_during:
                    cancel_pub=node.create_publisher(HandymanMsg,'/handyman_test/search_cancel',10)
                    wait(lambda:(acceptance_pending.is_set() if args.stop_during=='late_accept' else len(goal_ids)==1)
                        and cancel_pub.get_subscription_count()==1)
                    if args.stop_during in ('cancel','late_accept'):
                        if args.stop_during=='cancel':
                            wait(lambda:any(json.loads(line).get('observer_reason')=='search_goal_bound'
                                for line in (out/'observer.jsonl').read_text().splitlines()))
                        wrong_pub=node.create_publisher(HandymanMsg,'/handyman_test/search_request',
                            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
                        wait(lambda:wrong_pub.get_subscription_count()>=3)
                        wrong_notice=HandymanMsg();wrong_notice.message='search_cancelled'
                        wrong_notice.detail=yaml.safe_dump(dict(schema='handyman-search-request-v1',task_id='f'*32))
                        wrong_pub.publish(wrong_notice)
                        for i in range(5):rclpy.spin_once(node,timeout_sec=.05)
                        assert observer.poll() is None,'wrong task stopped observer'
                        cancel_sent=time.monotonic()
                        if args.process_fault:
                            from search_process_watchdog import SearchProcessWatchdog
                            from search_process_supervisor import SearchProcessSupervisor
                            assert set(goal_ids)==active_goal_ids and fixture.poll() is None
                            if independent is not None:
                                wait(lambda:independent.goal_id==goal_ids[0])
                                # Another live goal on the same fake server must be untouched.
                                from rclpy.action import ActionClient
                                auxiliary=ActionClient(node,NavigateToPose,'/handyman_test/navigate_to_pose')
                                wait(auxiliary.server_is_ready)
                                other=NavigateToPose.Goal();other.pose.header.frame_id='map'
                                other.pose.pose.position.x=-100.;other.pose.pose.orientation.w=1.
                                accepted=auxiliary.send_goal_async(other)
                                wait(accepted.done)
                                assert accepted.result().accepted
                                unrelated_id=bytes(accepted.result().goal_id.uuid).hex()
                                wait(unrelated_active.is_set)
                            inflight_evidence=dict(active_before_fault=sorted(active_goal_ids))
                            watchdog=SearchProcessWatchdog(request['task_id'],fixture,
                                .3 if args.process_fault=='deadline' else 10.)
                            supervisor=SearchProcessSupervisor(node,watchdog,
                                request_topic='/handyman_test/search_request',
                                status_topic='/handyman_test/search_execution_status',goal_canceller=independent)
                            if args.process_fault=='crash':
                                # Only this test's own domain-73 C++ child, never production nodes.
                                fixture.kill();fixture.wait(timeout=3)
                                assert fixture.returncode!=0
                            wait(lambda:any(r['event']=='search_cancelled' and
                                r['data'].get('task_id')==request['task_id'] for r in requests))
                            inflight_evidence['fault']=watchdog.sample()
                        else:
                            send('Task_failed')
                        wait(lambda:any(r['state']=='cancelled' for r in consumer_rows()))
                        wait(lambda:observer.poll() is not None,seconds=2)
                        observer_cancel_latency=time.monotonic()-cancel_sent
                    else:
                        map_msg.data[0]=100 if map_msg.data[0]!=100 else 0
                        wait(lambda:any(r['state']=='map_verification_revoked' for r in consumer_rows()))
                    if not args.worker_status:
                        # Legacy isolated harness forwarding (not used by reporter tests).
                        wrong=HandymanMsg();wrong.message='cancel_search';wrong.detail='wrong-task'
                        cancel_pub.publish(wrong)
                        for i in range(5):rclpy.spin_once(node,timeout_sec=.05)
                        assert not cancel_requests,'wrong task cancelled navigation'
                        msg=HandymanMsg();msg.message='cancel_search';msg.detail=request['task_id']
                        cancel_pub.publish(msg)
                    if args.stop_during=='late_accept':
                        wait(lambda:request['task_id'] in cancel_acks)
                        assert not goal_ids and not cancel_requests
                        if args.worker_status:
                            assert not any(r['state']=='cancel_drained' for r in worker_reports)
                        timeline.append(dict(event='executor_cancel_ack_before_acceptance',time=time.monotonic()))
                        release_acceptance.set()
                    if args.process_fault=='crash':
                        assert not cancellation_reports
                        if independent is None:
                            assert not cancel_requests and not cancelled_ids
                            assert active_goal_ids==set(goal_ids), 'fake goal ended before crash safety assertion'
                            inflight_evidence['active_after_worker_exit']=sorted(active_goal_ids)
                        else:
                            wait(lambda:independent.finished)
                            expected_exact=('rejected_or_uuid_mismatch' if args.cancel_fault=='reject' else
                                'timeout' if args.cancel_fault else 'goal_cancel_confirmed')
                            assert independent.events[-1]['state']==expected_exact,independent.events
                            assert unrelated_active.is_set() and unrelated_id not in cancel_requests
                            if not args.cancel_fault:
                                assert cancelled_ids==goal_ids and not active_goal_ids
                            else:
                                assert not any(e['state']=='goal_cancel_confirmed' for e in independent.events)
                            inflight_evidence.update(independent_cancel=independent.events,
                                unrelated_goal_id=unrelated_id,unrelated_still_active=True)
                    elif args.cancel_fault:
                        expected_cancel_state=('cancel_rejected_or_not_acknowledged' if args.cancel_fault=='reject'
                            else 'cancel_confirmation_timeout')
                        wait(lambda:any(r['state']==expected_cancel_state for r in cancellation_reports))
                        assert not any(r['state']=='cancel_confirmed' for r in cancellation_reports)
                        if args.cancel_fault=='no_terminal':
                            assert any(r['state']=='cancel_accepted_waiting_terminal' for r in cancellation_reports)
                    else:
                        wait(lambda:len(goal_ids)==1 and cancelled_ids==goal_ids)
                        wait(lambda:any(r['state']=='cancel_confirmed' for r in cancellation_reports))
                    assert cancel_requests==([] if args.process_fault=='crash' and independent is None else goal_ids)
                    assert all(r['goal_id'] in goal_ids for r in cancellation_reports)
                    if args.worker_status:
                        expected='cancel_failed' if args.cancel_fault or args.process_fault else 'cancel_drained'
                        wait(lambda:any(r['state']==expected for r in worker_reports))
                        if args.cancel_fault: assert not any(r['state']=='cancel_drained' for r in worker_reports)
                        notice=next(r['data'] for r in requests if r['event']=='search_cancelled' and r['data']['task_id']==request['task_id'])
                        assert all(r['task_id']==request['task_id'] and r['cancel_id']==notice['cancel_id'] for r in worker_reports)
                        if args.process_fault:
                            assert notice['reason'].startswith('worker_fault:')
                            if args.process_fault=='crash':
                                assert not any(r['state']=='cancel_drained' for r in worker_reports)
                            # Reset protocol only after internal fault already triggered cancellation.
                            send('Task_failed')
                        send('Environment','LayoutA');send('Are_you_ready?')
                        for i in range(10):rclpy.spin_once(node,timeout_sec=.05)
                        assert events.count('I_am_ready')==(1 if args.cancel_fault or args.process_fault else 2)
                        if args.process_fault:
                            send('Instruction','Go to the kitchen, grasp the apple and bring it to the dining table.')
                            for i in range(5):rclpy.spin_once(node,timeout_sec=.05)
                            assert len(goal_ids)==1
                vision=node.create_publisher(String,'/handyman/vision/diagnostics',10)
                def publish_vision():
                    ns=node.get_clock().now().nanoseconds
                    row=dict(target=request['target'],geometry_verified=True,audit_sha256=AUDIT_SHA256,
                        rgb_stamp_ns=ns,depth_stamp_ns=ns,tf_wait_status='ready',tf_at_depth_stamp={},
                        quality=dict(stable=True,position_m=[1,2,3],frame_id='odom',target=request['target'],stamp_ns=ns),
                        inference=dict(detections=[dict(name=request['target'])],class_conflicts=[],
                            view_health=dict(schema='handyman-view-health-v1',data_usable=True)))
                    msg=String();msg.data=json.dumps(row);vision.publish(msg)
                vision_timer=node.create_timer(.1,publish_vision)
                wait(lambda:observer.poll() is not None,seconds=12)
                assert observer.returncode==0
                if args.worker_status and not args.stop_during:
                    assert fixture.poll() is None, 'worker exited before post-search cancellation'
                else:
                    fixture.wait(timeout=3)
                    assert (fixture.returncode!=0 if args.process_fault=='crash' else fixture.returncode==0)
                observer_rows=[json.loads(line) for line in (out/'observer.jsonl').read_text().splitlines()]
                expected_state=('cancelled' if args.stop_during in ('cancel','late_accept') else
                    'incomplete' if args.stop_during else 'found')
                assert observer_rows[-1]['state']==expected_state,observer_rows[-1]
                if expected_state=='cancelled':
                    assert any(r.get('observer_reason')=='matching_task_cancelled' for r in observer_rows)
                    assert not any('search_timeout' in r.get('failures',[]) for r in observer_rows)
                if args.stop_during:assert not any(r['state'] in ('observe','found') for r in observer_rows)
                assert len(goal_ids)==1
                assert [r['goal_id'] for r in observer_rows if r.get('observer_reason')=='search_goal_bound']==(
                    [] if args.stop_during=='late_accept' else goal_ids)
                if args.stop_during=='late_accept':
                    assert [event['event'] for event in timeline]==[
                        'executor_cancel_ack_before_acceptance','server_acceptance_released','server_cancel_received']
                if not args.stop_during:assert consumer_rows()[-1]['state']=='map_content_verified'
                assert all(not r['actionable'] for r in observer_rows)
                vision_timer.cancel()
            if args.map_change:
                map_msg.data[0]=100 if map_msg.data[0]!=100 else 0
                wait(lambda:any(r['state']=='map_verification_revoked' for r in consumer_rows()))
            if args.worker_status and not args.stop_during:
                # Navigation and observation have completed, but task ownership remains.
                assert observer_rows[-1]['state']=='found'
                wrong_pub=node.create_publisher(HandymanMsg,'/handyman_test/search_request',
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
                wait(lambda:wrong_pub.get_subscription_count()>=3)
                wrong=HandymanMsg();wrong.message='search_cancelled'
                wrong.detail=yaml.safe_dump(dict(schema='handyman-search-request-v1',
                    task_id='f'*32,cancel_id='e'*32))
                wrong_pub.publish(wrong)
                for i in range(10):rclpy.spin_once(node,timeout_sec=.05)
                assert fixture.poll() is None and not worker_reports and len(goal_ids)==1
            if args.stop_during not in ('cancel','late_accept'):send('Task_failed')
            wait(lambda:any(r['event']=='search_cancelled' and r['data']['task_id']==request['task_id'] for r in requests))
            cancel=next(r['data'] for r in requests if r['event']=='search_cancelled' and r['data']['task_id']==request['task_id'])
            assert cancel['task_id']==request['task_id']
            assert (cancel['reason'].startswith('worker_fault:') if args.process_fault else cancel['reason']=='task_failed')
            if args.worker_status and not args.stop_during:
                wait(lambda:any(r['state']=='cancel_drained' for r in worker_reports))
                assert all(r['task_id']==request['task_id'] and r['cancel_id']==cancel['cancel_id'] for r in worker_reports)
                assert not any(r['state']=='cancel_failed' for r in worker_reports)
                assert not cancel_requests and not cancelled_ids, 'finished goal was cancelled again'
                send('Environment','LayoutA');send('Are_you_ready?')
                wait(lambda:events.count('I_am_ready')==2)
                fixture.wait(timeout=4);assert fixture.returncode==0
                assert len(goal_ids)==1
            wait(lambda:any(r['state']==('cancel_recorded' if args.map_change or args.stop_during=='map' else 'cancelled') for r in consumer_rows()))
            assert not set(events)&{'Task_finished','Object_grasped','Does_not_exist','Give_up'}
            send('Mission_complete');process.wait(timeout=5)
            assert process.returncode==0
            consumer.wait(timeout=20 if args.execute_search else 10)
            assert consumer.returncode==0
            cr=consumer_rows()
            assert all(not r['actionable'] for r in cr)
            result=dict(passed=True,events=events,requests=requests,consumer=cr,goal_ids=goal_ids,
                cancel_requests=cancel_requests,cancelled_ids=cancelled_ids,
                cancellation_reports=cancellation_reports,cancel_fault=args.cancel_fault,
                worker_reports=worker_reports,
                process_fault=args.process_fault,worker_fault_notices=worker_fault_notices,
                inflight_evidence=inflight_evidence,
                timeline=timeline,
                observer_cancel_latency_s=observer_cancel_latency,
                observer_final=observer_rows[-1] if observer_rows else None,
                scope='test-only request -> map check -> real executor/fake Nav2 -> observer; no production navigation authorization')
            (out/'summary.json').write_text(json.dumps(result,indent=2))
            print(json.dumps(result),flush=True)
        finally:
            unrelated_release.set()
            release_acceptance.set()
            for child in search_processes:
                if child.poll() is None:child.terminate();child.wait(timeout=5)
            if process.poll() is None:process.terminate();process.wait(timeout=5)
            if consumer.poll() is None:consumer.terminate();consumer.wait(timeout=5)
            if fake_executor:
                fake_executor.shutdown();fake_thread.join(timeout=2);fake_server.destroy();fake_node.destroy_node()
            node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()

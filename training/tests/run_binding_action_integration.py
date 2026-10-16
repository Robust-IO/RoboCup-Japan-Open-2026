"""Domain-73-only real C++ callback -> binding topic -> Python observer test."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import argparse


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--case',choices=('found','route','retry','retry_recover','retry_reorder'),default='found')
    case=parser.parse_args().case
    assert os.environ.get('ROS_DOMAIN_ID')=='73' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import yaml
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.executors import MultiThreadedExecutor
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from std_msgs.msg import String
    from handyman_msgs.msg import HandymanMsg
    from action_msgs.msg import GoalStatusArray
    from rclpy.qos import QoSProfile,DurabilityPolicy
    root=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(root/'training/scripts'))
    from rgbd_localization import AUDIT_SHA256
    config=root/'src/handyman_rebuild_ros2/config'
    content=(config/'environments/layout_a.yaml').read_bytes()
    env=yaml.safe_load(content);pose=env['rooms']['kitchen']['search_points'][0]
    digest=hashlib.sha256(content).hexdigest() # Test config digest, not production map-bundle proof.
    task=uuid.uuid4().hex;point=env['internal_name']+'/kitchen/0'
    out=Path(tempfile.mkdtemp(prefix='binding-action-'))
    print('OUTPUT',out,flush=True)
    rclpy.init();node=rclpy.create_node('fake_binding_nav2')
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    broadcaster=TransformBroadcaster(node)
    positions=[];bindings=[];handles=[]
    current_pose=dict(env['rooms']['bedroom']['search_points'][0] if case=='route' else pose)
    def execute(handle):
        handles.append(bytes(handle.goal_id.uuid).hex())
        positions.append(handle.request.pose.pose)
        requested=handle.request.pose.pose
        current_pose.update(x=requested.position.x,y=requested.position.y,
            yaw=math.atan2(2*requested.orientation.w*requested.orientation.z,
                          1-2*requested.orientation.z**2))
        time.sleep(.5)
        if case in ('retry','retry_recover','retry_reorder') and len(handles)==1:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()
    server=ActionServer(node,NavigateToPose,'/handyman_test/navigate_to_pose',execute_callback=execute)
    sub=node.create_subscription(HandymanMsg,'/handyman_test/search_binding',lambda m:bindings.append(yaml.safe_load(m.detail)),10)
    # Delay only the observer's status stream; C++ still receives actual action results.
    relay_state=dict(latest=None,release_at=None)
    qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    relay=node.create_publisher(GoalStatusArray,'/handyman_test/delayed_status',qos)
    def status_receive(msg):relay_state['latest']=msg
    status_sub=node.create_subscription(GoalStatusArray,'/handyman_test/navigate_to_pose/_action/status',status_receive,qos)
    def relay_tick():
        latest=relay_state['latest']
        if latest is None:return
        if len(bindings)>=2 and relay_state['release_at'] is None:
            relay_state['release_at']=time.monotonic()+.3
        release=relay_state['release_at'] is not None and time.monotonic()>=relay_state['release_at']
        msg=GoalStatusArray()
        msg.status_list=[s for s in latest.status_list if release or not (
            handles and bytes(s.goal_info.goal_id.uuid).hex()==handles[0] and s.status==6)]
        relay.publish(msg)
    relay_timer=node.create_timer(.05,relay_tick)
    vision=node.create_publisher(String,'/handyman/vision/diagnostics',10)
    def tf_tick():
        stamp=node.get_clock().now().to_msg()
        a=TransformStamped();a.header.stamp=stamp;a.header.frame_id='map';a.child_frame_id='odom';a.transform.rotation.w=1.
        b=TransformStamped();b.header.stamp=stamp;b.header.frame_id='odom';b.child_frame_id='base_footprint'
        b.transform.translation.x=float(current_pose['x']);b.transform.translation.y=float(current_pose['y'])
        b.transform.rotation.z=math.sin(float(current_pose['yaw'])/2);b.transform.rotation.w=math.cos(float(current_pose['yaw'])/2)
        broadcaster.sendTransform([a,b])
    timer=node.create_timer(.1,tf_tick)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    processes=[];logs=[]
    try:
        log=(out/'observer.console').open('w');logs.append(log)
        observer=subprocess.Popen([sys.executable,str(root/'training/scripts/search_arrival_ros.py'),
            '--binding-topic','/handyman_test/search_binding','--task-id',task,'--point-id',point,
            '--map-sha256',digest,'--x',str(pose['x']),'--y',str(pose['y']),'--yaw',str(pose['yaw']),
            '--motion-source','tf','--seconds','20','--action-status-topic',
            '/handyman_test/delayed_status' if case=='retry_reorder' else '/handyman_test/navigate_to_pose/_action/status',
            '--max-bound-attempts','2' if case in ('retry_recover','retry_reorder') else '1',
            '--output',str(out/'observer.jsonl')],stdout=log,stderr=subprocess.STDOUT)
        processes.append(observer)
        deadline=time.monotonic()+5
        while node.count_subscribers('/handyman_test/search_binding')<2 and time.monotonic()<deadline:time.sleep(.05)
        assert node.count_subscribers('/handyman_test/search_binding')>=2,'observer not ready'
        log=(out/'fixture.console').open('w');logs.append(log)
        fixture=subprocess.Popen(['/tmp/handyman-binding-fixture-build/search_binding_fixture',str(config/'environments.yaml'),
            'LayoutA',task,point,digest],stdout=log,stderr=subprocess.STDOUT)
        processes.append(fixture)
        deadline=time.monotonic()+22
        while observer.poll() is None and time.monotonic()<deadline:
            ns=node.get_clock().now().nanoseconds
            row=dict(target='canned_juice',geometry_verified=True,audit_sha256=AUDIT_SHA256,
                rgb_stamp_ns=ns,depth_stamp_ns=ns,tf_wait_status='ready',tf_at_depth_stamp={},
                quality=dict(stable=True,position_m=[1,2,3],frame_id='odom',target='canned_juice',stamp_ns=ns),
                inference=dict(detections=[dict(name='canned_juice')],class_conflicts=[],
                    view_health=dict(schema='handyman-view-health-v1',data_usable=True)))
            msg=String();msg.data=json.dumps(row);vision.publish(msg);time.sleep(.1)
        assert observer.poll()==0,'observer did not exit successfully'
        fixture.wait(timeout=5)
        rows=[json.loads(line) for line in (out/'observer.jsonl').read_text().splitlines()]
        if case in ('retry','retry_recover','retry_reorder'):
            assert len(bindings)==2 and len(handles)==2,(bindings,handles)
            assert handles[0]!=handles[1]
            assert [b['goal_id'] for b in bindings]==handles
            if case=='retry':
                assert rows[-1]['state']=='incomplete',rows[-1]
                assert not any(r['state'] in ('observe','found') for r in rows)
                assert [r['goal_id'] for r in rows if r.get('observer_reason')=='search_goal_bound']==handles[:1]
            else:
                assert rows[-1]['state']=='found',rows[-1]
                assert [r['goal_id'] for r in rows if r.get('observer_reason')=='search_goal_bound']==handles
                rebind=next(i for i,r in enumerate(rows) if r.get('attempt')==2)
                assert not any(r['state'] in ('observe','found') for r in rows[:rebind])
                assert any(r.get('motion',{}).get('reason')=='collecting_fresh_tf_window' for r in rows[rebind:])
                if case=='retry_reorder':
                    deferred=next(i for i,r in enumerate(rows) if r.get('observer_reason')=='retry_binding_deferred')
                    failed=next(i for i,r in enumerate(rows) if r.get('observer_reason')=='waiting_for_new_retry_binding')
                    assert deferred<failed<rebind
        else:
            assert rows[-1]['state']=='found',rows[-1]
            expected=1
            if case=='route':
                route=next(r for r in env['routes'] if r['from']=='bedroom' and r['to']=='kitchen')
                expected+=len(route['waypoints'])
                assert expected>1
                for actual,wanted in zip(positions,route['waypoints']):
                    assert abs(actual.position.x-wanted['x'])<1e-6 and abs(actual.position.y-wanted['y'])<1e-6
            assert len(bindings)==1 and len(handles)==expected,(bindings,handles)
            assert bindings[0]['goal_id']==handles[-1]
            assert [r['goal_id'] for r in rows if r.get('observer_reason')=='search_goal_bound']==handles[-1:]
        assert all(not r.get('actionable') and not r.get('does_not_exist_authorized') for r in rows)
        result=dict(passed=True,case=case,goal_ids=handles,bound_ids=[b['goal_id'] for b in bindings],final=rows[-1],
            scope='synthetic ROS; retry_recover uses fresh observation objects after ABORTED only')
        (out/'summary.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result),flush=True)
    finally:
        for process in processes:
            if process.poll() is None:process.terminate();process.wait(timeout=5)
        for log in logs:log.close()
        executor.shutdown();thread.join(timeout=2);server.destroy();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()

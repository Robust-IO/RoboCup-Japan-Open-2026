"""Isolated real coordinator cancellation dispatch barrier test."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',choices=('confirmed','rejected','timeout'),required=True)
    a=p.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='73' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import rclpy,yaml
    from rclpy.action import ActionServer,CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from handyman_msgs.msg import HandymanMsg
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    root=Path(__file__).resolve().parents[2]
    env=yaml.safe_load((root/'src/handyman_rebuild_ros2/config/environments/layout_a.yaml').read_text())
    pose=env['rooms']['bedroom']['search_points'][0]
    out=Path(tempfile.mkdtemp(prefix='coordinator-barrier-'));print('OUTPUT',out,flush=True)
    rclpy.init();node=rclpy.create_node('barrier_moderator');server_node=rclpy.create_node('barrier_fake_nav2')
    goals=[];events=[];cancel_seen=threading.Event();release=threading.Event()
    def execute(handle):
        goals.append(bytes(handle.goal_id.uuid).hex())
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            if release.is_set():
                if handle.is_cancel_requested:handle.canceled()
                else:handle.abort()
                return NavigateToPose.Result()
            time.sleep(.01)
        handle.abort();return NavigateToPose.Result()
    def cancel(handle):
        cancel_seen.set()
        return CancelResponse.REJECT if a.case=='rejected' else CancelResponse.ACCEPT
    server=ActionServer(server_node,NavigateToPose,'/handyman_test/barrier_nav',execute_callback=execute,
        cancel_callback=cancel,callback_group=ReentrantCallbackGroup())
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(server_node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    pub=node.create_publisher(HandymanMsg,'/handyman_test/barrier_in',10)
    sub=node.create_subscription(HandymanMsg,'/handyman_test/barrier_out',lambda m:events.append(m.message),10)
    broadcaster=TransformBroadcaster(node)
    def tick():
        tf=TransformStamped();tf.header.stamp=node.get_clock().now().to_msg();tf.header.frame_id='map';tf.child_frame_id='base_footprint'
        tf.transform.translation.x=float(pose['x']);tf.transform.translation.y=float(pose['y']);tf.transform.rotation.w=1.
        broadcaster.sendTransform(tf)
    timer=node.create_timer(.05,tick)
    def send(event,detail=''):
        msg=HandymanMsg();msg.message=event;msg.detail=detail;pub.publish(msg)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def wait(predicate,seconds=4):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return
        raise AssertionError(dict(goals=goals,events=events))
    with (out/'console.log').open('w') as log:
        proc=subprocess.Popen(['/tmp/handyman-search-nav-install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2/handyman_coordinator',
            '--ros-args','-p','navigation.action_name:=/handyman_test/barrier_nav',
            '-r','/handyman/message/to_robot:=/handyman_test/barrier_in',
            '-r','/handyman/message/to_moderator:=/handyman_test/barrier_out'],stdout=log,stderr=subprocess.STDOUT)
        try:
            wait(lambda:pub.get_subscription_count()>0);spin(.3)
            for i in range(15):
                send('Environment','LayoutA');send('Are_you_ready?');spin(.1)
                if 'I_am_ready' in events:break
            assert events.count('I_am_ready')==1
            instruction='Go to the kitchen, grasp the apple and bring it to the dining table.'
            send('Instruction',instruction);wait(lambda:len(goals)==1)
            send('Task_failed');wait(cancel_seen.is_set)
            send('Environment','LayoutA');send('Are_you_ready?');send('Instruction',instruction);spin(.2)
            assert len(goals)==1 and events.count('I_am_ready')==1
            if a.case=='confirmed':
                release.set()
                for i in range(15):
                    send('Are_you_ready?');spin(.05)
                    if events.count('I_am_ready')==2:break
                assert events.count('I_am_ready')==2
            else:
                spin(1.2);release.set();spin(.2)
                send('Are_you_ready?');send('Instruction',instruction);spin(.2)
                assert len(goals)==1 and events.count('I_am_ready')==1
            send('Mission_complete');proc.wait(timeout=3);assert proc.returncode==0
            text=(out/'console.log').read_text()
            assert 'Ready handshake blocked' in text
            expected={'confirmed':'cancel_confirmed','rejected':'cancel_rejected_or_not_acknowledged','timeout':'cancel_confirmation_timeout'}[a.case]
            assert expected in text
            result=dict(passed=True,case=a.case,goals=goals,events=events,scope='coordinator-owned navigation only')
            (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        finally:
            release.set()
            if proc.poll() is None:proc.terminate();proc.wait(timeout=3)
            executor.shutdown();thread.join(timeout=2);server.destroy();server_node.destroy_node();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()

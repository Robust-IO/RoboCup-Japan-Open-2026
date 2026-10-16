"""Coordinator receiver contract test. Fake worker; no Nav2 actions or Unity."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', choices=('confirmed', 'failed', 'timeout',
        'process_exit', 'process_zero', 'process_timeout',
        'supervised_exit', 'supervised_zero', 'supervised_timeout'), required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '73'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    import yaml
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from handyman_msgs.msg import HandymanMsg
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root/'training/scripts'))
    from search_process_watchdog import SearchProcessWatchdog
    worker = None
    watchdog = None
    supervisor = None
    env = yaml.safe_load((root/'src/handyman_rebuild_ros2/config/environments/layout_a.yaml').read_text())
    pose = env['rooms']['kitchen']['search_points'][0]
    out = Path(tempfile.mkdtemp(prefix='external-cancel-barrier-'))
    print('OUTPUT', out, flush=True)
    rclpy.init()
    node = rclpy.create_node('external_cancel_contract_test')
    events, requests = [], []
    pub = node.create_publisher(HandymanMsg, '/handyman_test/external_in', 10)
    status = node.create_publisher(HandymanMsg, '/handyman_test/external_status', 10)
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs = [node.create_subscription(HandymanMsg, '/handyman_test/external_out', lambda m: events.append(m.message), 10),
            node.create_subscription(HandymanMsg, '/handyman_test/external_request', lambda m: requests.append(m), qos)]
    broadcaster = TransformBroadcaster(node)
    def tick():
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id = 'map'; tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = float(pose['x']); tf.transform.translation.y = float(pose['y'])
        tf.transform.rotation.w = 1.
        broadcaster.sendTransform(tf)
    timer = node.create_timer(.05, tick)
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
    def wait(predicate):
        end = time.monotonic()+5
        while not predicate():
            assert time.monotonic() < end, (events, [m.message for m in requests])
            spin(.02)
    def send(event, detail=''):
        msg = HandymanMsg(); msg.message = event; msg.detail = detail; pub.publish(msg)
    def report(state, task=None, cancel=None):
        row = dict(schema='handyman-search-cancel-status-v1', task_id=task or cancellation['task_id'],
                   cancel_id=cancel or cancellation['cancel_id'], state=state)
        msg = HandymanMsg(); msg.message = 'search_cancel_status'; msg.detail = yaml.safe_dump(row)
        status.publish(msg)
    with (out/'console.log').open('w') as log:
        proc = subprocess.Popen([
            '/tmp/handyman-search-nav-install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2/handyman_coordinator',
            '--ros-args', '-p', 'search.publish_requests:=true',
            '-p', 'navigation.action_name:=/handyman_test/unused_external_nav',
            '-r', '/handyman/message/to_robot:=/handyman_test/external_in',
            '-r', '/handyman/message/to_moderator:=/handyman_test/external_out',
            '-r', '/handyman/search/request:=/handyman_test/external_request',
            '-r', '/handyman/search/execution_status:=/handyman_test/external_status'],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            wait(lambda: pub.get_subscription_count() and status.get_subscription_count())
            spin(.4)
            for _ in range(15):
                send('Environment', 'LayoutA'); send('Are_you_ready?'); spin(.1)
                if events.count('I_am_ready'): break
            assert events.count('I_am_ready') == 1
            send('Instruction', 'Go to the kitchen, grasp the apple and bring it to the dining table.')
            wait(lambda: any(m.message == 'search_requested' for m in requests))
            if args.case.startswith(('process_', 'supervised_')):
                request = yaml.safe_load(next(m.detail for m in requests if m.message == 'search_requested'))
                if args.case.startswith('supervised_'):
                    wrong = HandymanMsg(); wrong.message = 'search_worker_fault'
                    wrong.detail = yaml.safe_dump(dict(schema='handyman-search-worker-fault-v1',
                        task_id='f'*32, reason='worker_exited_without_retirement', returncode=7))
                    status.publish(wrong); spin(.2)
                    assert not any(m.message == 'search_cancelled' for m in requests)
                code = ('import time; time.sleep(60)' if args.case.endswith('_timeout') else
                        'import sys; sys.exit(7)' if args.case.endswith('_exit') else 'pass')
                worker = subprocess.Popen([sys.executable, '-c', code], stdout=log, stderr=subprocess.STDOUT)
                watchdog = SearchProcessWatchdog(request['task_id'], worker, .3)
                if args.case.startswith('supervised_'):
                    from search_process_supervisor import SearchProcessSupervisor
                    supervisor = SearchProcessSupervisor(node, watchdog,
                        request_topic='/handyman_test/external_request', status_topic='/handyman_test/external_status')
                else:
                    wait(lambda: watchdog.sample() is not None)
                    assert watchdog.failure_report() is None, 'must await matching cancellation context'
                if args.case.endswith('_timeout'): assert worker.poll() is None
            if supervisor is None:
                send('Task_failed')
            wait(lambda: any(m.message == 'search_cancelled' for m in requests))
            cancellation = yaml.safe_load(next(m.detail for m in requests if m.message == 'search_cancelled'))
            assert len(cancellation['cancel_id']) == 32
            if supervisor is not None:
                assert cancellation['reason'].startswith('worker_fault:')
                wait(lambda: watchdog.cancel_id is not None)
                assert watchdog.failure_report()['state'] == 'cancel_failed'
                # Only AFTER observing internally triggered cancellation, emulate
                # moderator reset so a ready rejection tests the barrier, not FSM state.
                send('Task_failed'); spin(.1)
            # Task_failed clears the environment; next-round protocol supplies it again.
            send('Environment', 'LayoutA'); spin(.1)
            report('cancel_drained', task='f'*32)
            report('cancel_drained', cancel='e'*32)
            report('cancel_received')
            bad = HandymanMsg(); bad.message = 'search_cancel_status'; bad.detail = '['; status.publish(bad)
            spin(.1); send('Are_you_ready?'); spin(.1)
            assert events.count('I_am_ready') == 1
            if args.case == 'failed': report('cancel_failed'); spin(.1)
            if args.case == 'timeout': spin(3.2)
            if watchdog is not None and supervisor is None:
                assert not watchdog.cancellation(dict(cancellation, task_id='f'*32))
                assert watchdog.failure_report() is None
                assert watchdog.cancellation(cancellation)
                failure = watchdog.failure_report()
                assert failure['state'] == 'cancel_failed'
                message = HandymanMsg(); message.message = 'search_cancel_status'
                message.detail = yaml.safe_dump(failure); status.publish(message); spin(.1)
                if args.case == 'process_timeout': assert worker.poll() is None
            report('cancel_drained'); spin(.1)
            send('Are_you_ready?'); spin(.2)
            assert events.count('I_am_ready') == (2 if args.case == 'confirmed' else 1)
            assert sum(m.message == 'search_requested' for m in requests) == 1
            send('Mission_complete'); proc.wait(timeout=3); assert proc.returncode == 0
            result = dict(passed=True, case=args.case, events=events,
                          scope='coordinator receiver with simulated worker status; no physical stop proof')
            if watchdog is not None:
                result.update(fault=watchdog.sample(), scope='real child process + watchdog + coordinator; no Nav2 goals')
                if supervisor is not None:
                    assert sum(m.message == 'search_cancelled' for m in requests) == 1
                    assert not set(events) & {'Give_up', 'Does_not_exist', 'Task_finished'}
                    result.update(automatic_cancel=True, cancellation_reason=cancellation['reason'])
            (out/'summary.json').write_text(json.dumps(result, indent=2))
            print(json.dumps(result), flush=True)
        finally:
            if worker is not None and worker.poll() is None:
                worker.terminate(); worker.wait(timeout=3)
            if proc.poll() is None: proc.terminate(); proc.wait(timeout=3)
            node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()

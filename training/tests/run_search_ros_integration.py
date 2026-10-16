"""Synthetic ROS integration, domain 73 only; never connects to Unity.

Run with system Python after sourcing ROS Humble. No action servers or velocity
command publishers. Output is a new temporary directory for every invocation.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import argparse


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-source',choices=('odom','tf'),default='odom')
    parser.add_argument('--tf-recovery-cases',action='store_true')
    args=parser.parse_args()
    if args.tf_recovery_cases and args.motion_source!='tf':
        parser.error('recovery cases require tf')
    if os.environ.get('ROS_DOMAIN_ID') != '73' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires isolated ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from action_msgs.msg import GoalStatusArray, GoalStatus
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    scripts=Path(__file__).resolve().parents[1]/'scripts'
    sys.path.insert(0,str(scripts))
    from rgbd_localization import AUDIT_SHA256
    out=Path(tempfile.mkdtemp(prefix='search-ros-integration-'))
    print('OUTPUT',out,flush=True)
    rclpy.init(); node=Node('synthetic_search_test')
    status=node.create_publisher(GoalStatusArray,'/navigate_to_pose/_action/status',
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom=node.create_publisher(Odometry,'/odom',10)
    vision=node.create_publisher(String,'/handyman/vision/diagnostics',10)
    broadcaster=TransformBroadcaster(node)
    results=[]
    process=None
    try:
        cases=('transient_gap','persistent_gap') if args.tf_recovery_cases else ('found','moving','outage','old_success','missing_tf')
        for index,case in enumerate(cases,1):
            key=f'{index:032x}'
            path=out/(case+'.jsonl')
            with (out/(case+'.console.log')).open('w') as console:
                process=subprocess.Popen([sys.executable,str(scripts/'search_arrival_ros.py'),
                    '--goal-id',key,'--x','0','--y','0','--yaw','0','--seconds','20',
                    '--motion-source',args.motion_source,
                    '--output',str(path)]+(['--recover-transient-tf'] if args.tf_recovery_cases else []),stdout=console,stderr=subprocess.STDOUT)
                began=time.monotonic(); pending=[]; observed=None; last_emit=0; connected=None
                while process.poll() is None and time.monotonic()-began < 25:
                    rclpy.spin_once(node,timeout_sec=.01)
                    now=time.monotonic(); elapsed=now-began
                    if connected is None and status.get_subscription_count() and (args.motion_source=='tf' or odom.get_subscription_count()):
                        connected=now
                        print(case,'DDS discovered after',round(elapsed,2),'seconds',flush=True)
                    rows=[json.loads(l) for l in path.read_text().splitlines() if l] if path.exists() else []
                    if observed is None and any(r.get('arrival_reason')=='arrived_and_settled' for r in rows):
                        observed=now
                    if now-last_emit < .1: continue
                    last_emit=now
                    item=GoalStatus(); item.goal_info.goal_id.uuid=list(bytes.fromhex(key))
                    item.status=4 if case=='old_success' or (connected is not None and now-connected>1) else 2
                    batch=GoalStatusArray();batch.status_list=[item];status.publish(batch)
                    stamp=node.get_clock().now().to_msg()
                    transform=TransformStamped();transform.header.stamp=stamp
                    transform.header.frame_id='map';transform.child_frame_id='base_footprint'
                    transform.transform.rotation.w=1.
                    if args.motion_source=='tf':
                        transform.header.frame_id='odom'
                        if case=='moving' and observed is not None:
                            transform.transform.translation.x=.1*(now-observed)
                        map_tf=TransformStamped();map_tf.header.stamp=stamp
                        map_tf.header.frame_id='map';map_tf.child_frame_id='odom'
                        map_tf.transform.rotation.w=1.
                        gap=(observed is not None and (case in ('outage','persistent_gap') or
                             (case=='transient_gap' and now-observed<.65)))
                        if not gap:
                            broadcaster.sendTransform([transform] if case=='missing_tf' else [map_tf,transform])
                    elif case!='missing_tf': broadcaster.sendTransform(transform)
                    msg=Odometry();msg.header.stamp=stamp;msg.header.frame_id='odom'
                    msg.child_frame_id='base_footprint';msg.pose.pose.orientation.w=1.
                    if case=='moving' and observed is not None: msg.twist.twist.linear.x=.1
                    # Publish TF first, then the corresponding odometry next cycle.
                    pending.append(msg)
                    if len(pending)>1:
                        old=pending.pop(0)
                        if args.motion_source=='odom' and not (case=='outage' and observed is not None): odom.publish(old)
                    if observed is not None and (case=='found' or (case in ('transient_gap','persistent_gap') and now-observed>.35)):
                        ns=stamp.sec*1_000_000_000+stamp.nanosec
                        row=dict(target='canned_juice',geometry_verified=True,audit_sha256=AUDIT_SHA256,
                            rgb_stamp_ns=ns,depth_stamp_ns=ns,tf_wait_status='ready',tf_at_depth_stamp={},
                            quality=dict(stable=True,position_m=[1,2,3],frame_id='odom',target='canned_juice',stamp_ns=ns),
                            inference=dict(detections=[dict(name='canned_juice')],class_conflicts=[],
                                view_health=dict(schema='handyman-view-health-v1',data_usable=True)))
                        v=String();v.data=json.dumps(row);vision.publish(v)
                if process.poll() is None:
                    process.terminate();process.wait(timeout=5)
                    raise AssertionError(case+' did not exit')
                rows=[json.loads(l) for l in path.read_text().splitlines()]
                final=rows[-1]
                assert process.returncode==0,(case,'exit',process.returncode)
                assert all(not r.get('actionable') and not r.get('does_not_exist_authorized') for r in rows)
                if case in ('found','transient_gap'):
                    assert final['state']=='found',final
                    if case=='transient_gap':
                        arrivals=[r for r in rows if r.get('arrival_reason')=='arrived_and_settled']
                        assert len(arrivals)==2,arrivals
                        assert arrivals[1]['arrival_stamp_ns']>arrivals[0]['arrival_stamp_ns']
                        assert any(r.get('observer_reason')=='tf_reacquiring' for r in rows)
                        assert final['tf_recovery_used'] is True
                else:
                    assert final['state']=='incomplete',final
                    assert not any(r['state']=='found' for r in rows)
                    reason={'moving':'movement_during_observation','outage':'odometry_stream_timeout',
                            'missing_tf':'exact_stamp_tf_unavailable'}.get(case)
                    if args.motion_source=='tf':
                        reason={'moving':'motion_or_invalid_tf_during_observation','outage':'tf_stream_timeout',
                                'persistent_gap':'tf_stream_timeout',
                                'missing_tf':'exact_stamp_tf_unavailable_or_invalid'}.get(case)
                    if reason: assert any(r.get('observer_reason')==reason for r in rows),(case,rows[-3:])
                    if case=='old_success': assert observed is None
                results.append(dict(case=case,passed=True,final=final))
                print(case,'PASS',flush=True)
        (out/'summary.json').write_text(json.dumps(results,indent=2))
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=5)
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__': main()

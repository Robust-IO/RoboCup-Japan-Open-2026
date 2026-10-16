"""Synthetic head feedback/RGB test. Domain 73 only; no Unity or base commands."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('isolated domain 73 required')
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState,Image
    from trajectory_msgs.msg import JointTrajectory
    rclpy.init();node=Node('head_view_fixture')
    joints=node.create_publisher(JointState,'/hsrb/joint_states',10)
    images=node.create_publisher(Image,'/hsrb/head_rgbd_sensor/rgb/image_raw',10)
    received=[]
    node.create_subscription(JointTrajectory,'/hsrb/head_trajectory_controller/command',received.append,10)
    out=Path(tempfile.mkdtemp(prefix='head-view-test-'));results=[];process=None
    print('OUTPUT',out,flush=True)
    try:
        for case in ('settled','wrong_feedback','no_feedback'):
            received.clear();dest=out/case
            with (out/(case+'.log')).open('x') as log:
                process=subprocess.Popen([sys.executable,str(Path(__file__).resolve().parents[1]/'scripts/head_view_trial.py'),
                    '--run','--confirm-head-motion','--synthetic-test','--output',str(dest)],stdout=log,stderr=subprocess.STDOUT)
                started=time.monotonic();last=0.
                while process.poll() is None and time.monotonic()-started<16:
                    rclpy.spin_once(node,timeout_sec=.01)
                    if time.monotonic()-last<.08:continue
                    last=time.monotonic();stamp=node.get_clock().now().to_msg()
                    if case!='no_feedback':
                        m=JointState();m.header.stamp=stamp;m.name=['head_pan_joint','head_tilt_joint']
                        m.position=[0.,-.5 if received and case=='settled' else 0.];joints.publish(m)
                    image=Image();image.header.stamp=stamp;image.width=8;image.height=8
                    image.encoding='rgb8';image.step=24;image.data=[128]*192;images.publish(image)
                if process.poll() is None:
                    process.terminate();process.wait(timeout=5);raise AssertionError('fixture timeout')
            result=json.loads((dest/'summary.json').read_text())
            assert len(received)==(0 if case=='no_feedback' else 1),(case,len(received))
            assert result['passed']==(case=='settled'),result
            assert result['frames']==(3 if case=='settled' else 0),result
            assert process.returncode==(0 if case=='settled' else 1)
            if received:
                assert list(received[0].joint_names)==['head_pan_joint','head_tilt_joint']
                assert list(received[0].points[0].positions)==[0.,-.5]
                assert received[0].points[0].time_from_start.sec==2
            results.append(dict(case=case,passed=True,commands=len(received),result=result))
            print(case,'PASS',flush=True)
        (out/'summary.json').write_text(json.dumps(results,indent=2))
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=5)
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()

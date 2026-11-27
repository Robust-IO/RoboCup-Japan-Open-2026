"""Explicit tucked-arm trial: 3cm lift OR small empty-gripper opening/return."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

NAMES=['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint']


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--gripper',action='store_true');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    names=NAMES+(['hand_motor_joint'] if a.gripper else [])
    if not a.run: raise SystemExit('No motion: requires --run')
    if a.output.exists(): raise SystemExit('Output exists')
    a.output.mkdir(parents=True)
    rclpy.init();node=rclpy.create_node('handyman_arm_lift_trial')
    latest={};samples=[];commands=[];start=None
    def cb(msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        values=dict(zip(msg.name,msg.position))
        if not all(n in values and math.isfinite(values[n]) for n in names):return
        latest.update(recv=now,stamp=stamp,positions=[float(values[n]) for n in names])
        samples.append(dict(latest))
    sub=node.create_subscription(JointState,'/hsrb/joint_states',cb,qos_profile_sensor_data)
    topic='/hsrb/gripper_controller/command' if a.gripper else '/hsrb/arm_trajectory_controller/command'
    pub=None
    def fresh():
        if not latest or time.monotonic()-latest['recv']>.4:raise RuntimeError('stale_feedback')
        age=node.get_clock().now().nanoseconds/1e9-latest['stamp']
        if not -.05<=age<=.5:raise RuntimeError('capture_time_not_fresh')
    def move(positions):
        fresh()
        if node.count_subscribers(topic)!=1 or node.count_publishers(topic)!=1:raise RuntimeError('control_ownership_changed')
        msg=JointTrajectory();msg.joint_names=['hand_motor_joint'] if a.gripper else NAMES
        point=JointTrajectoryPoint();point.positions=positions[-1:] if a.gripper else positions;point.time_from_start.sec=2;msg.points=[point]
        before=latest['stamp'];pub.publish(msg);commands.append(dict(positions=positions,duration=2))
        deadline=time.monotonic()+6;settled=[]
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1);fresh()
            if latest['stamp']<=before:continue
            before=latest['stamp']
            if max(abs(x-y) for x,y in zip(latest['positions'],positions))<.003:
                settled.append(latest['recv'])
                if len(settled)>=4 and settled[-1]-settled[0]>=.5:return
            else:settled=[]
        raise RuntimeError('motion_not_verified')
    result={}
    try:
        end=time.monotonic()+4
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        fresh()
        if len({s['stamp'] for s in samples})<5:raise RuntimeError('insufficient_fresh_feedback')
        start=latest['positions'][:]
        if abs(start[0])>.003 or abs(start[1])>.03 or abs(start[2])>.03 or abs(start[3]+math.pi/2)>.03 or abs(start[4])>.03:
            raise RuntimeError('not_initial_tucked_pose')
        if a.gripper and abs(start[5])>.02:raise RuntimeError('gripper_not_initial_zero')
        if node.count_publishers(topic)!=0 or node.count_subscribers(topic)!=1:raise RuntimeError('control_not_exclusive')
        pub=node.create_publisher(JointTrajectory,topic,10)
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        goal=start[:];goal[5 if a.gripper else 0]=.2 if a.gripper else .03
        move(goal);result['open_verified' if a.gripper else 'raise_verified']=True
        move(start);result['return_verified']=True
        result['status']='passed'
    except Exception as e:
        result.update(status='failed',reason=str(e))
        # No automatic return on stale feedback. Last command has a finite goal.
    finally:
        result.update(initial=start,commands=commands,samples=samples)
        (a.output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
        node.destroy_node();rclpy.shutdown()
    if result['status']!='passed':raise SystemExit(1)


if __name__=='__main__':main()

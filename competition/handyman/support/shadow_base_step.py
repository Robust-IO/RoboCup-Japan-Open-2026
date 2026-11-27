"""Live read-only adapter exercise. Deliberately has no publisher or --run mode.

Collision clearance is NOT inferred from missing depth. No live motion permission
is produced until the receiver-side stopping contract is established.
"""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer,TransformListener
from bounded_base_step import BaseStep
from finite_base_step_plan import plan


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    rclpy.init();n=rclpy.create_node('base_step_shadow');b=Buffer();listener=TransformListener(b,n);data={}
    subs=[n.create_subscription(JointState,'/hsrb/joint_states',lambda m:data.update(joints=m),qos_profile_sensor_data),
          n.create_subscription(LaserScan,'/hsrb/base_scan',lambda m:data.update(scan=m),qos_profile_sensor_data)]
    rows=[];last_pose=None;deadline=time.monotonic()+8
    names=['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']
    goal=[0,0,0,-math.pi/2,0,.65]
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.02)
        if len(data)!=2:continue
        try:
            tf=b.lookup_transform('odom','base_footprint',Time());t=tf.transform.translation;q=tf.transform.rotation
            pose=(t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            last_pose=pose
            j=data['joints'];positions=dict(zip(j.name,j.position))
            tucked=all(j.name.count(k)==1 and math.isfinite(positions[k]) and abs(positions[k]-v)<.005 for k,v in zip(names,goal))
            age=lambda stamp:(n.get_clock().now()-Time.from_msg(stamp)).nanoseconds/1e9
            ages=dict(tf_age=age(tf.header.stamp),scan_age=age(data['scan'].header.stamp),joints_age=age(j.header.stamp))
            exclusive=n.count_publishers('/hsrb/command_velocity')==0 and n.count_subscribers('/hsrb/command_velocity')==1
            now=time.monotonic();controller=BaseStep(pose,(.019,-.006),now)
            decision=controller.update(pose,now,**ages,exclusive=exclusive,tucked=tucked,collision_clear=False)
            rows.append(dict(ages=ages,exclusive=exclusive,tucked=tucked,decision=decision))
        except Exception as e:rows.append(dict(error=str(e)))
    report=dict(read_only=True,commands_sent=0,motion_authorized=False,samples=len(rows),
        valid_samples=sum('error' not in row for row in rows),
        nonzero_outputs=sum(any(row.get('decision',{}).get(k,0)!=0 for k in ('vx','vy','wz')) for row in rows),
        velocity_publishers=[e.node_name for e in n.get_publishers_info_by_topic('/hsrb/command_velocity')],
        omni_topics=[(name,types) for name,types in n.get_topic_names_and_types() if 'omni' in name],
        reason='Clearance intentionally withheld; receiver watchdog not established. This is fail-closed adapter verification, not motion validation.',rows=rows)
    report['omni_publishers']=[e.node_name for e in n.get_publishers_info_by_topic('/hsrb/omni_base_controller/command')]
    report['omni_subscribers']=n.count_subscribers('/hsrb/omni_base_controller/command')
    if last_pose is not None:report['finite_proposal']=plan(last_pose)
    with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}));n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()

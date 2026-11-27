"""Read-only reproduce arm startup predicates and image-time TF availability."""
import json,math,sys,time
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from apple_transfer_automation import RobotLock
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer,TransformListener

root=Path('/__HANDYMAN_PACKAGE__/runtime/can06-arm-startup-audit-01');root.mkdir(exist_ok=False)
with RobotLock():
    rclpy.init();node=rclpy.create_node('can06_readonly_arm_startup_audit');buf=Buffer();listener=TransformListener(buf,node)
    latest={};rows=[];joints=[]
    def cb(msg):
        latest.update(stamp=Time.from_msg(msg.header.stamp),recv=time.monotonic())
        joints.append(dict(stamp_ns=latest['stamp'].nanoseconds,received_monotonic=time.monotonic()))
    sub=node.create_subscription(JointState,'/hsrb/joint_states',cb,qos_profile_sensor_data)
    begin=time.monotonic();since=None;max_streak=0.;by_drain=[]
    try:
        for phase,seconds in [('single_spin',7.),('drain_callbacks',4.)]:
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.02)
                if phase=='drain_callbacks':
                    for _ in range(30):rclpy.spin_once(node,timeout_sec=0.)
                now=node.get_clock().now().nanoseconds
                row=dict(phase=phase,t=time.monotonic()-begin,has_joints=bool(latest),subscribers={
                    key:node.count_subscribers('/hsrb/'+key) for key in ('arm_trajectory_controller/command','gripper_controller/command')})
                row['map_ready']=buf.can_transform('map','base_footprint',Time())
                if latest:
                    row.update(joint_age_s=(now-latest['stamp'].nanoseconds)/1e9,
                        palm_at_latest_joint_ready=buf.can_transform('base_link','hand_palm_link',latest['stamp']),
                        latest_palm_ready=buf.can_transform('base_link','hand_palm_link',Time()))
                    if row['latest_palm_ready']:
                        stamp=Time.from_msg(buf.lookup_transform('base_link','hand_palm_link',Time()).header.stamp).nanoseconds
                        row['palm_lead_behind_joint_s']=(stamp-latest['stamp'].nanoseconds)/1e9
                    ready=(time.monotonic()-latest['recv']<=.4 and -.02<=row['joint_age_s']<=.4
                        and all(x==1 for x in row['subscribers'].values()) and row['map_ready'] and row['palm_at_latest_joint_ready'])
                    if ready:
                        if since is None:since=time.monotonic()
                        max_streak=max(max_streak,time.monotonic()-since)
                    else:since=None
                    row['original_ready']=ready
                rows.append(row)
        result=dict(robot_commands_sent=False,rows=rows,joint_samples=len(joints),maximum_original_ready_streak_s=max_streak,
            summaries={phase:dict(samples=len(xs),ready=sum(x.get('original_ready',False) for x in xs),
                missing_latest_joint_tf=sum(not x.get('palm_at_latest_joint_ready',False) for x in xs),
                missing_map=sum(not x['map_ready'] for x in xs))
                for phase in ('single_spin','drain_callbacks') for xs in [[r for r in rows if r['phase']==phase]]})
        with (root/'result.json').open('x') as f:json.dump(result,f,indent=2)
        print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
        print(json.dumps(rows[-15:]),flush=True)
    finally:node.destroy_node();rclpy.shutdown()

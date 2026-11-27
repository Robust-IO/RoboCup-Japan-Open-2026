"""Five fresh read-only startup windows with actual current TF; no publishers."""
import json,math,sys,time
from pathlib import Path
code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from can_arm_startup import CurrentTransformReadiness
from transfer_efficiency import wait_ready
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import Buffer,TransformListener
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/can-startup-live-verification-20261023-01');root.mkdir(exist_ok=False)
with RobotLock():
    rclpy.init();node=rclpy.create_node('can_startup_readonly_live_verification');buf=Buffer();listener=TransformListener(buf,node)
    latest={};rows=[];states=[]
    def receive(msg):
        if len(msg.name)!=len(msg.position):return
        latest.update(stamp=Time.from_msg(msg.header.stamp),recv=time.monotonic(),values=dict(zip(msg.name,msg.position)))
    sub=node.create_subscription(JointState,'/hsrb/joint_states',receive,qos_profile_sensor_data)
    try:
        for index in range(5):
            gate=CurrentTransformReadiness();attempts=[0];original_missing=[0];begin=time.monotonic()
            def check():
                attempts[0]+=1
                if not latest:return False
                now=node.get_clock().now().nanoseconds
                if time.monotonic()-latest['recv']>.4 or not -.02<=(now-latest['stamp'].nanoseconds)/1e9<=.4:return False
                if not all(node.count_subscribers('/hsrb/'+key)==1 for key in ('arm_trajectory_controller/command','gripper_controller/command')):return False
                if not buf.can_transform('map','base_footprint',Time()):return False
                if not buf.can_transform('base_link','hand_palm_link',latest['stamp']):original_missing[0]+=1
                return gate.ready(latest['stamp'].nanoseconds,now,
                    lambda ns:buf.can_transform('base_link','hand_palm_link',Time(nanoseconds=ns)))
            wait_ready(check,lambda dt:rclpy.spin_once(node,timeout_sec=dt))
            now=node.get_clock().now().nanoseconds
            rows.append(dict(index=index,elapsed_s=time.monotonic()-begin,checks=attempts[0],
                latched_real_joint_tf_ns=gate.stamp_ns,latched_tf_age_s=(now-gate.stamp_ns)/1e9,
                latest_joint_ns=latest['stamp'].nanoseconds,original_latest_tf_missing_checks=original_missing[0]))
            print(json.dumps(rows[-1]),flush=True)
        state=latest['values'];expected=dict(arm_lift_joint=0.,arm_flex_joint=0.,arm_roll_joint=0.,wrist_flex_joint=-math.pi/2,wrist_roll_joint=0.,hand_motor_joint=0.)
        if any(abs(state[k]-v)>.005 for k,v in expected.items()):raise ValueError('robot_changed_from_empty_tucked_pose')
        report=dict(status='five_current_can_startup_windows_verified_without_motion',rows=rows,
            robot_commands_sent=False,source_age_limit_s=.4,startup_stability_window_s=.15)
        write_json(root/'result.json',report);print(json.dumps(report),flush=True)
    finally:node.destroy_node();rclpy.shutdown()

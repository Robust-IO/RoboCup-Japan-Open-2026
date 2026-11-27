"""Bounded read-only ROS state probe, used before navigation validation."""
import os,subprocess,time,json,math
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='71':
    cmd='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/navigation-efficiency-probe-roslogs; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(Path(__file__).resolve())
    raise SystemExit(subprocess.run(['bash','-lc',cmd]).returncode)
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState,LaserScan
from tf2_ros import Buffer,TransformListener
rclpy.init();n=rclpy.create_node('navigation_efficiency_readonly_probe');b=Buffer();listener=TransformListener(b,n);data={}
subs=[n.create_subscription(kind,topic,lambda m,k=k:data.update({k:m}),qos_profile_sensor_data) for k,kind,topic in [('joints',JointState,'/hsrb/joint_states'),('scan',LaserScan,'/hsrb/base_scan')]]
report=dict(robot_commands_sent=False,status='not_connected');start=time.monotonic()
while time.monotonic()-start<8:
    rclpy.spin_once(n,timeout_sec=.02)
    try:
        if len(data)!=2:continue
        tf=b.lookup_transform('map','base_footprint',Time());now=n.get_clock().now().nanoseconds
        ages={k:(now-Time.from_msg(v.header.stamp).nanoseconds)/1e9 for k,v in data.items()};ages['tf']=(now-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
        if not all(-.05<=v<=.6 for v in ages.values()):continue
        q=tf.transform.rotation;t=tf.transform.translation;j=dict(zip(data['joints'].name,data['joints'].position))
        values=[j[k] for k in ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')]
        report.update(status='connected',pose=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],joints=values,ages_s=ages,checked_ns=now,
            command_publishers={s:n.count_publishers(s) for s in ['/hsrb/command_velocity','/hsrb/omni_base_controller/command','/hsrb/arm_trajectory_controller/command','/hsrb/gripper_controller/command']})
        break
    except Exception as e:report['last_error']=str(e)
report['wait_s']=time.monotonic()-start
p=Path('/__HANDYMAN_PACKAGE__/runtime/navigation-efficiency-live-probe-latest.json');p.write_text(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown();print(json.dumps(report))

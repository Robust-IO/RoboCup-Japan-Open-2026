"""Read-only ROS interface probe: no publishers or action clients."""
import json
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Image


def main():
    rclpy.init()
    node=rclpy.create_node('handyman_grasp_interface_probe')
    stats={}; joints={}; subscriptions=[]
    def callback(topic,msg):
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        s=stats.setdefault(topic,dict(count=0,distinct_stamps=set()))
        s['count']+=1;s['distinct_stamps'].add(stamp);s['last_receive']=time.monotonic()
        s['frame_id']=msg.header.frame_id
        if isinstance(msg,JointState):
            joints.update({name:float(value) for name,value in zip(msg.name,msg.position)})
        else: s.update(width=msg.width,height=msg.height,encoding=msg.encoding)
    topics={'/hsrb/joint_states':JointState,'/hsrb/head_rgbd_sensor/rgb/image_raw':Image,
            '/hsrb/head_rgbd_sensor/depth_registered/image_raw':Image,'/hsrb/hand_camera/image_raw':Image}
    for topic,kind in topics.items():
        subscriptions.append(node.create_subscription(kind,topic,lambda msg,t=topic:callback(t,msg),qos_profile_sensor_data))
    end=time.monotonic()+10
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    controls=['/hsrb/arm_trajectory_controller/command','/hsrb/gripper_controller/command']
    endpoints={t:dict(subscribers=node.count_subscribers(t),publishers=node.count_publishers(t)) for t in controls}
    now=time.monotonic()
    for topic in topics:
        s=stats.setdefault(topic,dict(count=0,distinct_stamps=set()))
        s['distinct_stamps']=len(s['distinct_stamps'])
        if 'last_receive' in s:s['receive_age_seconds']=round(now-s.pop('last_receive'),3)
    expected=['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']
    result=dict(read_only=True,streams=stats,control_endpoints=endpoints,joints={k:joints.get(k) for k in expected},
                missing_or_invalid_joints=[k for k in expected if k not in joints or not math.isfinite(joints[k])],
                note='Recent reception and moving timestamps do not alone verify capture-time freshness or controller readiness.')
    print(json.dumps(result,allow_nan=False,indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()

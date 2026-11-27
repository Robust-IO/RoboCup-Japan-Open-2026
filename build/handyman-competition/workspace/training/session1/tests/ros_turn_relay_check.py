"""Isolated ROS-domain integration check. Never run on the robot's domain71."""
import os,time,tempfile,json
from pathlib import Path
import numpy as np
if os.environ.get('ROS_DOMAIN_ID')!='183' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise RuntimeError('isolated_test_domain183_required')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist,TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener,TransformBroadcaster
from navigation_turn_relay import NavigationTurnRelay,RAW_TOPIC,OUTPUT_TOPIC
from carry_clearance import CarryClearance
from candidate_selection import CandidateSelector
rclpy.init();node=rclpy.create_node('isolated_turn_guard_check')
relay=None
try:
 with tempfile.TemporaryDirectory() as tmp:
    root=Path(tmp);grid=np.ones((160,160),bool);grid[65:69,92:96]=False
    np.savez(root/'map.npz',free=grid,resolution=.025,origin=[0,0,0],radius=.2)
    geometry=CarryClearance(CandidateSelector(grid,.025,[0,0,0],.2,{}),np.array([[.65,-.04,.3],[.65,.04,.3]]))
    buffer=Buffer();listener=TransformListener(buffer,node);broadcaster=TransformBroadcaster(node)
    relay=NavigationTurnRelay(node,buffer,dict(polygon=geometry.polygon.tolist(),map_archive=str(root/'map.npz')),[2,1,-1.57],lambda e:None)
    # Heading to goal is right. Static obstacle blocks that arc; left is clear.
    raw=node.create_publisher(Twist,RAW_TOPIC,1);scanpub=node.create_publisher(LaserScan,'/hsrb/base_scan',qos_profile_sensor_data)
    output=[];sub=node.create_subscription(Twist,OUTPUT_TOPIC,lambda m:output.append((m.linear.x,m.linear.y,m.angular.z)),10)
    def send():
        stamp=node.get_clock().now().to_msg()
        tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='map';tf.child_frame_id='base_footprint'
        tf.transform.translation.x=2.;tf.transform.translation.y=2.;tf.transform.rotation.w=1.;broadcaster.sendTransform(tf)
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_footprint';scan.angle_min=-3.14;scan.angle_increment=.01;scan.range_min=.05;scan.range_max=20.;scan.ranges=[float('inf')]*629;scanpub.publish(scan)
        msg=Twist();msg.linear.x=.2;msg.angular.z=-.3;raw.publish(msg)
    timer=node.create_timer(.05,send)
    start=time.monotonic()
    while time.monotonic()-start<.6:rclpy.spin_once(node,timeout_sec=.02)
    relay.enable();start=time.monotonic()
    while time.monotonic()-start<2.5:rclpy.spin_once(node,timeout_sec=.02)
    assert relay.error is None,relay.error
    assert any(w>.01 for x,y,w in output), 'no_left_turn_forwarded'
    assert all(w>=0 for x,y,w in output), 'unsafe_right_turn_leaked'
    assert all(abs(x)<1e-9 and abs(y)<1e-9 for x,y,w in output), 'translation_leaked_during_directed_turn'
    node.destroy_timer(timer);start=time.monotonic()
    while time.monotonic()-start<.5:rclpy.spin_once(node,timeout_sec=.02)
    assert output[-1]==(0.,0.,0.), 'stale_input_did_not_stop'
    relay.close();relay=None
    print(json.dumps(dict(status='passed',domain=183,output_messages=len(output),robot_domain_used=False)))
finally:
 if relay is not None:relay.close()
 node.destroy_node();rclpy.shutdown()

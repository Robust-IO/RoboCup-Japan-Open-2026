"""One explicitly requested staging goal. No arm/gripper or velocity publisher."""
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not a.run:raise SystemExit('No motion without --run')
    if a.output.exists():raise ValueError('Output exists')
    rclpy.init();node=rclpy.create_node('phase5_staging_goal');buf=Buffer();listener=TransformListener(buf,node)
    scan={};report={'target':dict(x=3.35,y=.35,yaw=-math.pi/2),'arm_control':False};handle=None
    def cb(m):scan.update(message=m,received=time.monotonic())
    sub=node.create_subscription(LaserScan,'/hsrb/base_scan',cb,qos_profile_sensor_data)
    client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    def wait(f,seconds):
        end=time.monotonic()+seconds
        while not f.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if not f.done():raise RuntimeError('service_response_timeout')
        return f.result()
    def check():
        if not scan or time.monotonic()-scan['received']>.5:raise RuntimeError('stale_laser')
        m=scan['message'];age=(node.get_clock().now()-Time.from_msg(m.header.stamp)).nanoseconds/1e9
        if not -.05<=age<=.5:raise RuntimeError('stale_laser_capture')
        ranges=[v for v in m.ranges if math.isfinite(v) and m.range_min<=v<=m.range_max]
        if len(ranges)<20:raise RuntimeError('insufficient_laser_returns')
        tf=buf.lookup_transform('map','base_footprint',Time())
        if (node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds>500_000_000:raise RuntimeError('stale_pose')
        return tf
    try:
        end=time.monotonic()+4
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        tf=check()
        if math.hypot(tf.transform.translation.x-1.184,tf.transform.translation.y)>.15:raise RuntimeError('initial_pose_changed')
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('navigation_unavailable')
        g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=node.get_clock().now().to_msg()
        g.pose.pose.position.x=3.35;g.pose.pose.position.y=.35;g.pose.pose.orientation.z=math.sin(-math.pi/4);g.pose.pose.orientation.w=math.cos(-math.pi/4)
        handle=wait(client.send_goal_async(g),5)
        if not handle.accepted:raise RuntimeError('goal_rejected')
        print('Goal accepted; monitoring laser and TF freshness',flush=True)
        result=handle.get_result_async()
        while not result.done():
            rclpy.spin_once(node,timeout_sec=.1);check()
        r=result.result();tf=check();t=tf.transform.translation;q=tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        error=math.hypot(t.x-3.35,t.y-.35);angle=abs(math.atan2(math.sin(yaw+math.pi/2),math.cos(yaw+math.pi/2)))
        report.update(action_status=r.status,final_pose=dict(x=t.x,y=t.y,yaw=yaw),position_error=error,yaw_error=angle,
                      success=r.status==4 and error<=.12 and angle<=.15)
    except Exception as e:
        report.update(success=False,error=str(e))
        if handle is not None and handle.accepted:
            try:report['cancel_requested']=True;wait(handle.cancel_goal_async(),3)
            except Exception as cancel_error:report['cancel_error']=str(cancel_error)
    finally:
        a.output.write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
        node.destroy_node();rclpy.shutdown()
    if not report['success']:raise SystemExit(1)


if __name__=='__main__':main()

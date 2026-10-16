"""Bounded TF-derived odometry probe. Default: preflight, no ROS nodes.

--run logs only. --run --publish additionally publishes a fixed shadow topic;
never /odom, TF, velocity commands, navigation actions or competition messages.
"""
import argparse
import json
import math
from pathlib import Path
import time
from tf_derived_odometry import TfDerivedOdometry, OdometryDiagnostics


def planar_transform(transform):
    t,q=transform.translation,transform.rotation
    values=(t.x,t.y,t.z,q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>.002:
        raise ValueError('invalid_transform')
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    if abs(roll)>.05 or abs(pitch)>.05 or abs(t.z)>.05:raise ValueError('nonplanar_base_transform')
    return dict(x=t.x,y=t.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--publish',action='store_true')
    p.add_argument('--observe-commands',action='store_true',help='Log received velocity commands; never send commands')
    p.add_argument('--seconds',type=float,default=10.)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not math.isfinite(a.seconds) or not 0<a.seconds<=30:p.error('seconds must be (0,30]')
    if a.publish and not a.run:p.error('--publish requires --run')
    if not a.run:
        print('Preflight only: no nodes, files, or publishers. Unity may remain paused.');return
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import Twist
    with a.output.open('x') as stream:
        rclpy.init();node=rclpy.create_node('handyman_derived_odometry_probe')
        estimator=TfDerivedOdometry();diagnostics=OdometryDiagnostics();counts=dict(valid=0,invalid=0,published=0);authority=None;conflict=False
        pub=node.create_publisher(Odometry,'/handyman/odometry/derived',10) if a.publish else None
        def emit(row):
            if row is None:return
            row=diagnostics.annotate(row)
            counts['valid' if row['valid'] else 'invalid']+=1
            if row['valid'] and pub is not None:
                msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
                msg.header.stamp.sec=row['stamp_ns']//1_000_000_000;msg.header.stamp.nanosec=row['stamp_ns']%1_000_000_000
                pose=row['pose'];msg.pose.pose.position.x=pose['x'];msg.pose.pose.position.y=pose['y']
                msg.pose.pose.orientation.z=math.sin(pose['yaw']/2);msg.pose.pose.orientation.w=math.cos(pose['yaw']/2)
                msg.twist.twist.linear.x=row['twist']['vx'];msg.twist.twist.linear.y=row['twist']['vy'];msg.twist.twist.angular.z=row['twist']['wz']
                # Conservative placeholders, not calibrated statistical uncertainty.
                for index in (0,7,14,21,28,35):
                    msg.pose.covariance[index]=1e6;msg.twist.covariance[index]=1e6
                pub.publish(msg);counts['published']+=1
            row['record_type']='estimate'
            row['receipt_monotonic_ns']=time.monotonic_ns()
            row['receipt_ros_ns']=node.get_clock().now().nanoseconds
            stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
        def receive(message,info):
            nonlocal authority,conflict
            # Humble also omits publisher_gid from take_message metadata.
            # Conservative fallback: require one publisher for the WHOLE /tf topic.
            endpoints=node.get_publishers_info_by_topic('/tf')
            if len(endpoints)!=1:
                conflict=True
                emit(estimator.invalidate('multiple_or_unknown_tf_publishers'));return
            gid=bytes(endpoints[0].endpoint_gid).hex()
            for tf in message.transforms:
                if (tf.header.frame_id,tf.child_frame_id)!=('odom','base_footprint'):continue
                if authority is None:authority=gid
                elif authority!=gid:conflict=True
                if conflict:emit(estimator.invalidate('multiple_tf_edge_publishers'));continue
                try:pose=planar_transform(tf.transform)
                except ValueError as exc:emit(estimator.invalidate(str(exc)));continue
                stamp=tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec
                emit(estimator.update(pose,stamp,node.get_clock().now().nanoseconds,time.monotonic()))
        sub=node.create_subscription(TFMessage,'/tf',receive,qos_profile_sensor_data)
        command_sub=(node.create_subscription(Twist,'/hsrb/command_velocity',lambda msg:None,
                     qos_profile_sensor_data) if a.observe_commands else None)
        try:
            deadline=time.monotonic()+a.seconds
            while rclpy.ok() and time.monotonic()<deadline:
                # This Humble executor drops MessageInfo before calling callbacks.
                # Take directly; authority is checked separately using the ROS graph.
                with sub.handle:
                    taken=sub.handle.take_message(TFMessage,False)
                if taken is not None:receive(*taken)
                else:time.sleep(.005)
                if command_sub is not None:
                    with command_sub.handle:
                        command=command_sub.handle.take_message(Twist,False)
                    if command is not None:
                        msg=command[0]
                        values=dict(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,
                                    wx=msg.angular.x,wy=msg.angular.y,wz=msg.angular.z)
                        row=dict(record_type='observed_command',receipt_monotonic_ns=time.monotonic_ns(),
                                 receipt_ros_ns=node.get_clock().now().nanoseconds,
                                 finite=all(math.isfinite(v) for v in values.values()),actionable=False)
                        row['command']={k:v if math.isfinite(v) else None for k,v in values.items()}
                        stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
                emit(estimator.tick(node.get_clock().now().nanoseconds,time.monotonic()))
        except (KeyboardInterrupt,ExternalShutdownException):pass
        finally:
            emit(estimator.invalidate('probe_stopped'))
            node.destroy_node();rclpy.try_shutdown()
            print(json.dumps(dict(output=str(a.output),counts=counts,nav2_connected=False,control_started=False)))


if __name__=='__main__':main()

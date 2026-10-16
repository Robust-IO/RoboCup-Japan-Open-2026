"""Bounded read-only /tf stationarity probe. No navigation or action publishers."""
import argparse
import json
import math
from pathlib import Path
import time
from tf_motion_gate import TfMotionGate


def planar_pose(transform):
    t, q = transform.translation, transform.rotation
    values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite_transform')
    if abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1) > .002:
        raise ValueError('invalid_quaternion')
    return dict(x=t.x, y=t.y,
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))


class MotionObserver:
    def __init__(self, now):
        self.gate = TfMotionGate()
        self.last_receive = now
        self.timed_out = False

    def receive(self, pose, stamp, sensor_now, now):
        self.last_receive = now
        self.timed_out = False
        return self.gate.update(pose, stamp, sensor_now, now)

    def tick(self, now):
        if now-self.last_receive > .3 and not self.timed_out:
            self.gate.reset()
            self.timed_out = True
            return dict(valid=False, stationary=False, actionable=False,
                        reason='tf_stream_timeout')
        return None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=8)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if not 0 < a.seconds <= 30:
        p.error('seconds must be in (0,30]')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    from rclpy.executors import ExternalShutdownException
    with a.output.open('x') as stream:
        rclpy.init()
        node = Node('handyman_tf_motion_readonly')
        observer = MotionObserver(time.monotonic())
        def emit(row):
            row.update(read_only=True, actionable=False, monotonic_s=time.monotonic())
            stream.write(json.dumps(row, allow_nan=False)+'\n')
            stream.flush()
        def receive(msg):
            for tf in msg.transforms:
                if (tf.header.frame_id,tf.child_frame_id) != ('odom','base_footprint'):
                    continue
                ns = tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec
                try:
                    pose = planar_pose(tf.transform)
                except ValueError as exc:
                    observer.gate.reset()
                    emit(dict(valid=False, stationary=False, reason=str(exc)))
                    continue
                emit(observer.receive(pose,ns,node.get_clock().now().nanoseconds,time.monotonic()))
        try:
            sub = node.create_subscription(TFMessage,'/tf',receive,qos_profile_sensor_data)
            end = time.monotonic()+a.seconds
            while rclpy.ok() and time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.02)
                row = observer.tick(time.monotonic())
                if row is not None:
                    emit(row)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            emit(dict(valid=False,stationary=False,reason='probe_stopped'))
            node.destroy_node()
            rclpy.try_shutdown()
    print(str(a.output),flush=True)


if __name__ == '__main__':
    main()

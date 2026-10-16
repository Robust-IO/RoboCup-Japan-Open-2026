#!/usr/bin/env python3
"""
Publish a placeholder odom->base_footprint dynamic TF at low frequency.

This allows Nav2 costmaps to initialize before the SIGVerse simulator
connects. Once the SIGVerse bridge starts publishing sensor data (LaserScan),
this node automatically stops to avoid TF conflicts with the bridge.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class PlaceholderTfPublisher(Node):
    def __init__(self):
        super().__init__('placeholder_tf_publisher')

        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('rate', 2.0)

        self.br = TransformBroadcaster(self)
        self._stopped = False
        self._parent = self.get_parameter('parent_frame').value
        self._child = self.get_parameter('child_frame').value

        # Detect bridge connection by subscribing to laser scan
        # (only published by the SIGVerse bridge when Unity is connected)
        self._scan_sub = self.create_subscription(
            LaserScan, '/hsrb/base_scan', self._scan_callback, 1)

        rate = self.get_parameter('rate').value
        self.timer = self.create_timer(1.0 / rate, self.publish_tf)
        self.get_logger().info(
            f'Publishing placeholder TF: {self._parent} -> {self._child} '
            f'at {rate} Hz (will stop when bridge takes over)')

    def _scan_callback(self, msg: LaserScan):
        """Stop publishing once the bridge is sending laser scans."""
        if self._stopped:
            return
        self.get_logger().info(
            'Bridge is active (received LaserScan), stopping placeholder TF.')
        self._stopped = True
        self.timer.cancel()
        # Unsubscribe to avoid unnecessary processing
        self.destroy_subscription(self._scan_sub)

    def publish_tf(self):
        if self._stopped:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._parent
        t.child_frame_id = self._child
        t.transform.translation.x = self.get_parameter('x').value
        t.transform.translation.y = self.get_parameter('y').value
        t.transform.translation.z = self.get_parameter('z').value

        yaw = self.get_parameter('yaw').value
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PlaceholderTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

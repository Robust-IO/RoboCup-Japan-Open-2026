"""Publish an exported Handyman semantic map as persistent RViz markers."""

import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker, MarkerArray

from handyman_map_tools.semantic_map import (
    SemanticMapError,
    load,
    point_xy,
    pose_xy_yaw,
    require_valid,
)


class SemanticMarkerNode(Node):
    """Load one JSON file and publish its geometry for visual inspection."""

    def __init__(self):
        super().__init__('handyman_semantic_marker')
        self.declare_parameter('input', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('republish_period', 2.0)
        input_path = self.get_parameter('input').value
        self._frame_id = self.get_parameter('frame_id').value
        republish_period = float(
            self.get_parameter('republish_period').value
        )
        if republish_period <= 0.0:
            raise SemanticMapError(
                'republish_period must be greater than zero'
            )
        if not input_path:
            raise SemanticMapError('the input parameter is required')
        self._document = load(input_path)
        result = require_valid(self._document)
        for warning in result.warnings:
            self.get_logger().warning(warning)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            MarkerArray, 'semantic_map_markers', qos
        )
        self._published_once = False
        self.add_on_set_parameters_callback(self._parameters_changed)
        self._timer = self.create_timer(
            republish_period, self._publish_markers
        )
        # Publish immediately for subscribers that are already available.
        self._publish_markers()

    def _parameters_changed(self, parameters):
        for parameter in parameters:
            if parameter.name != 'input':
                continue
            try:
                document = load(parameter.value)
                result = require_valid(document)
            except (SemanticMapError, OSError) as error:
                return SetParametersResult(
                    successful=False,
                    reason=str(error),
                )
            self._document = document
            self._published_once = False
            for warning in result.warnings:
                self.get_logger().warning(warning)
            self._publish_markers()
        return SetParametersResult(successful=True)

    def _publish_markers(self):
        message = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        message.markers.append(delete)

        marker_id = 0
        for room in self._document['rooms']:
            marker = self._base_marker('rooms', marker_id, Marker.LINE_STRIP)
            marker_id += 1
            marker.scale.x = 0.05
            marker.color.r = 0.1
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 1.0
            for value in room['polygon'] + room['polygon'][:1]:
                x, y = point_xy(value)
                marker.points.append(Point(x=x, y=y, z=0.04))
            marker.text = room['id']
            message.markers.append(marker)

            label = self._base_marker(
                'room_labels', marker_id, Marker.TEXT_VIEW_FACING
            )
            marker_id += 1
            coordinates = [point_xy(value) for value in room['polygon']]
            label.pose.position.x = (
                sum(item[0] for item in coordinates) / len(coordinates)
            )
            label.pose.position.y = (
                sum(item[1] for item in coordinates) / len(coordinates)
            )
            label.pose.position.z = 0.3
            label.scale.z = 0.28
            label.color.r = 0.1
            label.color.g = 0.7
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = room['id']
            message.markers.append(label)

        for destination in self._document['destinations']:
            x, y, yaw = pose_xy_yaw(destination['pose'])
            marker = self._base_marker('destinations', marker_id, Marker.CUBE)
            marker_id += 1
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.08
            marker.pose.orientation.z = math.sin(yaw * 0.5)
            marker.pose.orientation.w = math.cos(yaw * 0.5)
            bounds = destination.get('axis_aligned_bounds') or {}
            size = bounds.get('size') or {'x': 0.25, 'y': 0.25}
            size_x, size_y = point_xy(size, 'destination bounds size')
            marker.scale.x = max(abs(size_x), 0.1)
            marker.scale.y = max(abs(size_y), 0.1)
            marker.scale.z = 0.16
            marker.color.r = 1.0
            marker.color.g = 0.55
            marker.color.b = 0.05
            marker.color.a = 0.55
            message.markers.append(marker)

            label = self._base_marker(
                'destination_labels', marker_id, Marker.TEXT_VIEW_FACING
            )
            marker_id += 1
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.35
            label.scale.z = 0.18
            label.color.r = 1.0
            label.color.g = 0.75
            label.color.b = 0.1
            label.color.a = 1.0
            label.text = destination['id']
            message.markers.append(label)

        for candidate in self._document['object_spawn_candidates']:
            x, y, _ = pose_xy_yaw(candidate['pose'])
            marker = self._base_marker(
                'spawn_candidates', marker_id, Marker.SPHERE
            )
            marker_id += 1
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.08
            marker.scale.x = marker.scale.y = marker.scale.z = 0.10
            marker.color.r = 0.25
            marker.color.g = 1.0
            marker.color.b = 0.25
            marker.color.a = 0.8
            message.markers.append(marker)

        robot_pose = self._document.get('robot_initial_pose')
        if robot_pose is not None:
            x, y, yaw = pose_xy_yaw(robot_pose)
            marker = self._base_marker(
                'robot_initial_pose', marker_id, Marker.ARROW
            )
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.12
            marker.pose.orientation.z = math.sin(yaw * 0.5)
            marker.pose.orientation.w = math.cos(yaw * 0.5)
            marker.scale.x = 0.6
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 0.9
            marker.color.g = 0.1
            marker.color.b = 0.9
            marker.color.a = 1.0
            message.markers.append(marker)

        self._publisher.publish(message)
        if not self._published_once:
            self.get_logger().info(
                'Published semantic markers for '
                f"{self._document['environment']} on "
                '/semantic_map_markers; republishing periodically'
            )
            self._published_once = True

    def _base_marker(self, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=0.0).to_msg()
        return marker


def main(args=None):
    """Run the semantic marker publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticMarkerNode()
        rclpy.spin(node)
    except SemanticMapError as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'ERROR: {error}')
        raise SystemExit(2) from error
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

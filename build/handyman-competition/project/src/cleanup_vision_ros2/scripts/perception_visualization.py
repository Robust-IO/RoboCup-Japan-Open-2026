#!/usr/bin/env python3
"""Helpers for publishing RViz-friendly debug markers."""

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


HEAD_POINTING_LENGTH_M = 2.2
HAND_COMMAND_SCALE = 4.0


def _first_frame_id(*msgs):
    for msg in msgs:
        header = getattr(msg, 'header', None)
        frame_id = getattr(header, 'frame_id', '')
        if frame_id:
            return frame_id
    return 'map'


def _first_stamp(*msgs):
    for msg in msgs:
        header = getattr(msg, 'header', None)
        stamp = getattr(header, 'stamp', None)
        if stamp is not None:
            return stamp
    return None


def _color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _base_marker(frame_id, stamp, namespace, marker_id, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _delete_all_marker(frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp
    marker.action = Marker.DELETEALL
    marker.pose.orientation.w = 1.0
    return marker


def _text_marker(frame_id, stamp, namespace, marker_id, point, text, color):
    marker = _base_marker(frame_id, stamp, namespace, marker_id, Marker.TEXT_VIEW_FACING)
    marker.pose.position = point
    marker.scale.z = 0.12
    marker.color = color
    marker.text = text
    return marker


def build_head_marker_array(avatar_msg, scene_msg, pointing_msg):
    frame_id = _first_frame_id(scene_msg, avatar_msg, pointing_msg)
    stamp = _first_stamp(scene_msg, avatar_msg, pointing_msg)
    markers = MarkerArray()
    markers.markers.append(_delete_all_marker(frame_id, stamp))

    if getattr(avatar_msg, 'is_valid', False):
        origin = getattr(avatar_msg, 'origin_odom', Point())
        avatar_marker = _base_marker(frame_id, stamp, 'avatar', 0, Marker.SPHERE)
        avatar_marker.pose.position = origin
        avatar_marker.scale.x = 0.12
        avatar_marker.scale.y = 0.12
        avatar_marker.scale.z = 0.12
        avatar_marker.color = _color(0.1, 0.9, 0.9, 0.9)
        markers.markers.append(avatar_marker)

        avatar_text = _text_marker(
            frame_id,
            stamp,
            'avatar',
            1,
            Point(x=origin.x, y=origin.y, z=origin.z + 0.18),
            (
                f"avatar conf={getattr(avatar_msg, 'confidence', 0.0):.2f} "
                f"yaw={getattr(avatar_msg, 'body_yaw', 0.0):.2f}"
            ),
            _color(0.9, 1.0, 1.0, 0.95),
        )
        markers.markers.append(avatar_text)

    if getattr(pointing_msg, 'is_valid', False):
        origin = getattr(pointing_msg, 'origin', Point())
        direction = getattr(pointing_msg, 'direction', None)
        if direction is not None:
            arrow = _base_marker(frame_id, stamp, 'pointing', 0, Marker.ARROW)
            arrow.scale.x = 0.03
            arrow.scale.y = 0.06
            arrow.scale.z = 0.08
            arrow.color = _color(1.0, 0.2, 0.2, 0.95)
            arrow.points = [
                Point(x=origin.x, y=origin.y, z=origin.z),
                Point(
                    x=origin.x + direction.x * HEAD_POINTING_LENGTH_M,
                    y=origin.y + direction.y * HEAD_POINTING_LENGTH_M,
                    z=origin.z + direction.z * HEAD_POINTING_LENGTH_M,
                ),
            ]
            markers.markers.append(arrow)

    for index, obj in enumerate(getattr(scene_msg, 'objects', [])):
        if not getattr(obj, 'has_3d_position', False):
            continue

        position = getattr(obj, 'position', Point())
        sphere = _base_marker(frame_id, stamp, 'objects', index * 2, Marker.SPHERE)
        sphere.pose.position = position
        sphere.scale.x = 0.09
        sphere.scale.y = 0.09
        sphere.scale.z = 0.09
        sphere.color = _color(0.2, 1.0, 0.2, 0.9)
        markers.markers.append(sphere)

        label = _text_marker(
            frame_id,
            stamp,
            'objects',
            index * 2 + 1,
            Point(x=position.x, y=position.y, z=position.z + 0.14),
            (
                f"{getattr(obj, 'class_name', 'object')} "
                f"{getattr(obj, 'confidence', 0.0):.2f}"
            ),
            _color(0.8, 1.0, 0.8, 0.95),
        )
        markers.markers.append(label)

    return markers


def build_hand_marker_array(alignment_msg):
    frame_id = _first_frame_id(alignment_msg)
    stamp = _first_stamp(alignment_msg)
    markers = MarkerArray()
    markers.markers.append(_delete_all_marker(frame_id, stamp))

    base = Point(x=0.0, y=0.0, z=0.0)
    status_color = _color(0.1, 0.95, 0.2, 0.95) if getattr(
        alignment_msg, 'in_grasp_window', False) else _color(1.0, 0.75, 0.1, 0.95)

    if getattr(alignment_msg, 'is_target_found', False):
        tip = Point(
            x=getattr(alignment_msg, 'recommended_linear_x', 0.0) * HAND_COMMAND_SCALE,
            y=getattr(alignment_msg, 'recommended_linear_y', 0.0) * HAND_COMMAND_SCALE,
            z=getattr(alignment_msg, 'recommended_lift_delta', 0.0) * HAND_COMMAND_SCALE,
        )
        arrow = _base_marker(frame_id, stamp, 'hand_alignment', 0, Marker.ARROW)
        arrow.scale.x = 0.025
        arrow.scale.y = 0.05
        arrow.scale.z = 0.07
        arrow.color = status_color
        arrow.points = [base, tip]
        markers.markers.append(arrow)

        tip_marker = _base_marker(frame_id, stamp, 'hand_alignment', 1, Marker.SPHERE)
        tip_marker.pose.position = tip
        tip_marker.scale.x = 0.05
        tip_marker.scale.y = 0.05
        tip_marker.scale.z = 0.05
        tip_marker.color = status_color
        markers.markers.append(tip_marker)

    text = 'hand target: none'
    if getattr(alignment_msg, 'is_target_found', False):
        text = (
            f"{getattr(alignment_msg, 'target_class', 'target')} "
            f"conf={getattr(alignment_msg, 'confidence', 0.0):.2f} "
            f"err=({getattr(alignment_msg, 'pixel_error_x', 0.0):.2f},"
            f"{getattr(alignment_msg, 'pixel_error_y', 0.0):.2f}) "
            f"window={'yes' if getattr(alignment_msg, 'in_grasp_window', False) else 'no'}"
        )
    markers.markers.append(
        _text_marker(
            frame_id,
            stamp,
            'hand_status',
            0,
            Point(x=0.0, y=0.0, z=0.18),
            text,
            status_color if getattr(alignment_msg, 'is_target_found', False)
            else _color(1.0, 0.3, 0.3, 0.95),
        )
    )

    return markers

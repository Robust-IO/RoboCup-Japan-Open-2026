#!/usr/bin/env python3
"""
Hand-camera perception node for Interactive Cleanup.

Responsibilities:
  1. Detect near-field graspable objects from the hand camera.
  2. Publish object candidates in image space.
  3. Publish a conservative alignment recommendation for near-field servoing.
"""

import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge

from cleanup_vision_ros2.msg import HandTargetAlignment, SceneObject, SceneObjectArray
from perception_common import (
    HAND_ACTIVE_MODES,
    HAND_CAM_TOPIC,
    HAND_CAMERA_INFO_TOPIC,
    find_yolo_model_path,
    load_yolo_class,
)
from perception_visualization import build_hand_marker_array


class HandPerceptionNode(Node):
    def __init__(self):
        super().__init__('hand_perception_node')

        self.bridge = CvBridge()
        self.rgb_image = None
        self.img_w = 640
        self.img_h = 480
        self.current_mode = 'IDLE'

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, HAND_CAM_TOPIC, self._rgb_cb, sensor_qos)
        self.create_subscription(CameraInfo, HAND_CAMERA_INFO_TOPIC, self._info_cb, 10)
        self.create_subscription(String, '/cleanup_perception/mode', self._mode_cb, 10)

        self.objects_pub = self.create_publisher(
            SceneObjectArray, '/cleanup_perception/hand/objects', 10)
        self.alignment_pub = self.create_publisher(
            HandTargetAlignment, '/cleanup_perception/hand/target_alignment', 10)
        self.debug_pub = self.create_publisher(
            Image, '/cleanup_perception/debug/hand_image', 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/cleanup_perception/debug/hand_markers', 10)

        self.model = None
        self.class_names = {}
        yolo_class, yolo_error = load_yolo_class()
        if yolo_class is None:
            self.get_logger().warn(
                f'{yolo_error}. Hand object detection will remain disabled.')
        else:
            try:
                model_path = find_yolo_model_path()
                self.model = yolo_class(model_path)
                self.class_names = self.model.names
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            except Exception as exc:
                self.get_logger().error(
                    f'Failed to initialize YOLO model: {exc}. '
                    'Hand object detection will remain disabled.')

        self.last_detect_time = self.get_clock().now()
        self.detect_interval_sec = 0.15

        self.get_logger().info('HandPerceptionNode ready')

    def _rgb_cb(self, msg: Image):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.rgb_image = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
            self.img_h, self.img_w = self.rgb_image.shape[:2]
        except Exception as exc:
            self.get_logger().error(f'Hand RGB error: {exc}')

    def _info_cb(self, msg: CameraInfo):
        self.img_w = int(msg.width) if msg.width > 0 else self.img_w
        self.img_h = int(msg.height) if msg.height > 0 else self.img_h

    def _mode_cb(self, msg: String):
        self.current_mode = msg.data.strip() or 'IDLE'

    def _is_active(self):
        return self.current_mode in HAND_ACTIVE_MODES

    def run_detection(self):
        if not self._is_active() or self.rgb_image is None or self.model is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_detect_time).nanoseconds / 1e9
        if dt < self.detect_interval_sec:
            return
        self.last_detect_time = now

        bgr = self.rgb_image.copy()
        stamp = now.to_msg()

        scene_msg = SceneObjectArray()
        scene_msg.header.stamp = stamp
        scene_msg.header.frame_id = 'hand_camera_frame'
        scene_msg.source_camera = 'hand'

        align_msg = HandTargetAlignment()
        align_msg.header.stamp = stamp
        align_msg.header.frame_id = 'hand_camera_frame'
        align_msg.is_target_found = False
        align_msg.target_class = ''
        align_msg.confidence = 0.0

        preds = self.model.predict(bgr, conf=0.25, verbose=False)
        boxes = preds[0].boxes

        best = None
        best_score = float('-inf')

        for box in boxes:
            cls_id = int(box.cls[0].item())
            cls_name = self.class_names.get(cls_id, f'class_{cls_id}')
            if cls_name == 'person':
                continue

            conf = float(box.conf[0].item())
            cx, cy, bw, bh = [int(x) for x in box.xywh[0].tolist()]
            x1, y1, x2, y2 = [int(x) for x in box.xyxy[0].tolist()]

            obj = SceneObject()
            obj.class_name = cls_name
            obj.confidence = conf
            obj.bbox_cx = cx
            obj.bbox_cy = cy
            obj.bbox_w = bw
            obj.bbox_h = bh
            obj.has_3d_position = False
            obj.depth_mm = 0.0
            scene_msg.objects.append(obj)

            area_ratio = float(bw * bh) / max(float(self.img_w * self.img_h), 1.0)
            center_bias = abs(cx - self.img_w * 0.5) / max(self.img_w * 0.5, 1.0)
            score = conf + area_ratio * 0.5 - center_bias * 0.2
            if score > best_score:
                best_score = score
                best = {
                    'class_name': cls_name,
                    'confidence': conf,
                    'cx': cx,
                    'cy': cy,
                    'w': bw,
                    'h': bh,
                    'area_ratio': area_ratio,
                }

            cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(
                bgr, f'{cls_name} {conf:.2f}', (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        if best is not None:
            error_x = float(best['cx'] - self.img_w * 0.5) / max(self.img_w * 0.5, 1.0)
            error_y = float(best['cy'] - self.img_h * 0.5) / max(self.img_h * 0.5, 1.0)

            align_msg.is_target_found = True
            align_msg.target_class = best['class_name']
            align_msg.pixel_error_x = error_x
            align_msg.pixel_error_y = error_y
            align_msg.bbox_area_ratio = best['area_ratio']
            align_msg.in_grasp_window = (
                abs(error_x) <= 0.12 and
                abs(error_y) <= 0.18 and
                best['area_ratio'] >= 0.02
            )
            align_msg.recommended_linear_x = max(0.0, min(0.05, (0.05 - best['area_ratio']) * 1.2))
            align_msg.recommended_linear_y = max(-0.04, min(0.04, -error_x * 0.05))
            align_msg.recommended_lift_delta = max(-0.03, min(0.03, -error_y * 0.04))
            align_msg.confidence = best['confidence']

            cross_x = int(self.img_w * 0.5)
            cross_y = int(self.img_h * 0.5)
            cv2.line(bgr, (cross_x - 12, cross_y), (cross_x + 12, cross_y), (0, 0, 255), 2)
            cv2.line(bgr, (cross_x, cross_y - 12), (cross_x, cross_y + 12), (0, 0, 255), 2)

        self.objects_pub.publish(scene_msg)
        self.alignment_pub.publish(align_msg)
        self.marker_pub.publish(build_hand_marker_array(align_msg))

        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = HandPerceptionNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.run_detection()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

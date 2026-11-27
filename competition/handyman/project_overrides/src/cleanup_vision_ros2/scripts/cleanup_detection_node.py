#!/usr/bin/env python3
"""
cleanup_detection_node.py — Interactive Cleanup Vision Node (ROS 2 Humble)

Provides two capabilities for the Interactive Cleanup competition:
  1. Object detection via YOLO (COCO pretrained or custom weights)
     with depth-based 3D localization.
  2. Avatar pointing direction estimation via MediaPipe PoseLandmarker.

Published topics:
  /cleanup_vision/detected_objects   (cleanup_vision_ros2/DetectedObjectArray)
  /cleanup_vision/pointing_direction (cleanup_vision_ros2/PointingDirection)
  /cleanup_vision/debug_image        (sensor_msgs/Image)

Subscribed topics:
  /hsrb/head_rgbd_sensor/rgb/image_raw               (sensor_msgs/Image)
  /hsrb/head_rgbd_sensor/depth_registered/image_raw  (sensor_msgs/Image)
  /hsrb/head_rgbd_sensor/rgb/camera_info             (sensor_msgs/CameraInfo)
  /cleanup_vision/enable                             (std_msgs/Bool)
"""

import os
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge

import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_matrix

try:
    import mediapipe as mp
    HAS_MEDIAPIPE = True
except ImportError:
    mp = None
    HAS_MEDIAPIPE = False

try:
    from mediapipe.tasks.python import vision as mp_vision
    from mediapipe.tasks.python.core.base_options import BaseOptions
    from mediapipe.tasks.python.vision.core.vision_task_running_mode import (
        VisionTaskRunningMode,
    )
    HAS_MEDIAPIPE_TASKS = HAS_MEDIAPIPE
except (ImportError, AttributeError):
    mp_vision = None
    BaseOptions = None
    VisionTaskRunningMode = None
    HAS_MEDIAPIPE_TASKS = False

HAS_MEDIAPIPE_SOLUTIONS = bool(
    HAS_MEDIAPIPE and
    getattr(getattr(mp, 'solutions', None), 'pose', None) is not None
)

from cleanup_vision_ros2.msg import (
    DetectedObject,
    DetectedObjectArray,
    PointingDirection,
)
from perception_common import (
    CAMERA_INFO_TOPIC,
    DEPTH_CAM_TOPIC,
    HEAD_CAM_TOPIC,
    describe_pose_backend_unavailable as _describe_pose_backend_unavailable,
    find_pose_model_path as _find_pose_model_path,
    find_yolo_model_path as _find_yolo_model_path,
    landmark_visibility as _landmark_visibility,
    load_yolo_class as _load_yolo_class,
    select_pose_backend as _select_pose_backend,
)

# MediaPipe landmark indices
_L_SHOULDER, _L_ELBOW, _L_WRIST, _L_INDEX = 11, 13, 15, 19
_R_SHOULDER, _R_ELBOW, _R_WRIST, _R_INDEX = 12, 14, 16, 20

class CleanupDetectionNode(Node):
    def __init__(self):
        super().__init__('cleanup_detection_node')

        self.rgb_image = None
        self.depth_image = None
        self.bridge = CvBridge()

        # Camera intrinsics (filled from CameraInfo)
        self.fx = self.fy = self.cx_cam = self.cy_cam = None
        self.img_w = 640
        self.img_h = 480

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Detection gate
        self.detection_enabled = False

        # QoS for sensor streams
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, HEAD_CAM_TOPIC, self._rgb_cb, sensor_qos)
        self.create_subscription(
            Image, DEPTH_CAM_TOPIC, self._depth_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, 10)
        self.create_subscription(
            Bool, '/cleanup_vision/enable', self._enable_cb, 10)

        self.objects_pub = self.create_publisher(
            DetectedObjectArray, '/cleanup_vision/detected_objects', 10)
        self.pointing_pub = self.create_publisher(
            PointingDirection, '/cleanup_vision/pointing_direction', 10)
        self.debug_pub = self.create_publisher(
            Image, '/cleanup_vision/debug_image', 1)

        # YOLO
        self.model = None
        self.coco_names = {}
        yolo_class, yolo_error = _load_yolo_class()
        if yolo_class is None:
            self.get_logger().warn(
                f'{yolo_error}. Object detection will remain disabled.')
        else:
            try:
                model_path = _find_yolo_model_path()
                self.model = yolo_class(model_path)
                self.coco_names = self.model.names
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            except Exception as exc:
                self.get_logger().error(
                    f'Failed to initialize YOLO model: {exc}. '
                    'Object detection will remain disabled.')

        # MediaPipe PoseLandmarker
        self.pose_detector = None
        self.pose_timestamp_ms = 0
        self.pose_backend = 'disabled'
        pose_model_path = _find_pose_model_path()
        self.pose_backend = _select_pose_backend(
            HAS_MEDIAPIPE_TASKS,
            pose_model_path,
            HAS_MEDIAPIPE_SOLUTIONS,
        )
        if self.pose_backend == 'tasks':
            try:
                options = mp_vision.PoseLandmarkerOptions(
                    base_options=BaseOptions(model_asset_path=pose_model_path),
                    running_mode=VisionTaskRunningMode.VIDEO,
                    num_poses=1,
                    min_pose_detection_confidence=0.5,
                    min_pose_presence_confidence=0.5,
                    min_tracking_confidence=0.5,
                    output_segmentation_masks=False,
                )
                self.pose_detector = mp_vision.PoseLandmarker.create_from_options(
                    options)
                self.get_logger().info(
                    f'MediaPipe PoseLandmarker initialized: {pose_model_path}')
            except Exception as exc:
                self._init_pose_solution_fallback(
                    f'PoseLandmarker initialization failed: {exc}')
        elif self.pose_backend == 'solutions':
            if pose_model_path is None:
                self._init_pose_solution_fallback(
                    'PoseLandmarker model asset not found in '
                    'cleanup_vision_ros2/models/')
            else:
                self._init_pose_solution_fallback('MediaPipe Tasks unavailable')
        elif not HAS_MEDIAPIPE and not HAS_MEDIAPIPE_TASKS:
            self.get_logger().warn(
                'MediaPipe unavailable — pointing estimation disabled')
        else:
            self.get_logger().warn(
                f'{_describe_pose_backend_unavailable(HAS_MEDIAPIPE, HAS_MEDIAPIPE_TASKS, pose_model_path, HAS_MEDIAPIPE_SOLUTIONS)} '
                '— pointing estimation disabled')

        self._has_display = bool(os.environ.get('DISPLAY', ''))
        self.last_detect_time = self.get_clock().now()
        self.detect_interval_sec = 0.2  # 5 Hz

        self.get_logger().info('CleanupDetectionNode ready')

    def destroy_node(self):
        if self.pose_detector is not None:
            try:
                close_fn = getattr(self.pose_detector, 'close', None)
                if close_fn is not None:
                    close_fn()
            except Exception:
                pass
            self.pose_detector = None
        return super().destroy_node()

    def _init_pose_solution_fallback(self, reason: str):
        if not HAS_MEDIAPIPE_SOLUTIONS:
            self.pose_backend = 'disabled'
            self.get_logger().warn(f'{reason}. Pointing estimation disabled.')
            return

        try:
            self.pose_detector = mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=1,
                smooth_landmarks=True,
                enable_segmentation=False,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self.pose_backend = 'solutions'
            self.get_logger().warn(
                f'{reason}. Falling back to MediaPipe Pose solution.')
        except Exception as exc:
            self.pose_backend = 'disabled'
            self.pose_detector = None
            self.get_logger().warn(
                f'{reason}. MediaPipe Pose fallback failed: {exc}. '
                'Pointing estimation disabled.')

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------
    def _rgb_cb(self, msg: Image):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.rgb_image = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
            self.img_h, self.img_w = self.rgb_image.shape[:2]
        except Exception as e:
            self.get_logger().error(f'RGB error: {e}')

    def _depth_cb(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth error: {e}')

    def _info_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx_cam = msg.k[2]
            self.cy_cam = msg.k[5]
            self.get_logger().info(
                f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx_cam:.1f} cy={self.cy_cam:.1f}')

    def _enable_cb(self, msg: Bool):
        self.detection_enabled = msg.data
        state = 'ENABLED' if msg.data else 'DISABLED'
        self.get_logger().info(f'Detection {state}')

    # ------------------------------------------------------------------
    # 3-D helpers
    # ------------------------------------------------------------------
    def _pixel_to_cam3d(self, u, v):
        """Return (x, y, z, raw_depth) in camera frame, or None."""
        if self.depth_image is None or self.fx is None:
            return None

        h, w = self.depth_image.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None

        r = 3
        roi = self.depth_image[
            max(0, vi - r):min(h, vi + r + 1),
            max(0, ui - r):min(w, ui + r + 1)].astype(float)
        valid = roi[roi > 0]
        if len(valid) == 0:
            return None

        raw = float(np.median(valid))
        d = raw / 1000.0 if raw > 100 else raw

        x = (u - self.cx_cam) * d / self.fx
        y = (v - self.cy_cam) * d / self.fy
        return (x, y, d, raw)

    def _cam3d_to_odom(self, x, y, z):
        """Transform camera-frame point to odom frame.  Returns (x,y,z) or None."""
        # Try the proper camera optical frame first
        for frame in ('head_rgbd_sensor_rgb_frame',
                      'head_rgbd_sensor_link',
                      'rgbd_sensor_rgb_frame'):
            try:
                ts = self.tf_buffer.lookup_transform(
                    'odom', frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                t = ts.transform.translation
                r = ts.transform.rotation
                mat = quaternion_matrix([r.x, r.y, r.z, r.w])
                mat[0:3, 3] = [t.x, t.y, t.z]
                pt = mat @ np.array([x, y, z, 1.0])
                return (float(pt[0]), float(pt[1]), float(pt[2]))
            except Exception:
                continue

        # Fallback: rough estimate from base_footprint
        try:
            ts = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())
            tx = ts.transform.translation.x
            ty = ts.transform.translation.y
            rot = ts.transform.rotation
            _, _, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            mx = tx + z * math.cos(yaw) - x * math.sin(yaw)
            my = ty + z * math.sin(yaw) + x * math.cos(yaw)
            mz = 1.0 - y  # approximate head height ~1 m
            return (mx, my, mz)
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Avatar pointing estimation
    # ------------------------------------------------------------------
    def _estimate_pointing(self, bgr):
        if self.pose_detector is None:
            return None

        rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        if self.pose_backend == 'tasks':
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            now_ms = int(self.get_clock().now().nanoseconds / 1_000_000)
            self.pose_timestamp_ms = max(self.pose_timestamp_ms + 1, now_ms)

            try:
                results = self.pose_detector.detect_for_video(
                    mp_image, self.pose_timestamp_ms)
            except Exception as exc:
                self.get_logger().warn(f'PoseLandmarker inference failed: {exc}')
                return None

            landmark_sets = results.pose_landmarks
        else:
            try:
                results = self.pose_detector.process(rgb)
            except Exception as exc:
                self.get_logger().warn(
                    f'MediaPipe Pose fallback inference failed: {exc}')
                return None

            landmark_sets = []
            if results.pose_landmarks is not None:
                landmark_sets.append(results.pose_landmarks.landmark)

        if not landmark_sets:
            return None

        h, w = bgr.shape[:2]
        best = None
        best_score = -1.0
        for lm in landmark_sets:
            def _arm_ext(s, e, wr):
                vis = min(
                    _landmark_visibility(lm[s]),
                    _landmark_visibility(lm[e]),
                    _landmark_visibility(lm[wr]),
                )
                if vis < 0.3:
                    return None
                shoulder_wrist = math.hypot(
                    lm[wr].x - lm[s].x,
                    lm[wr].y - lm[s].y,
                )
                elbow_wrist = math.hypot(
                    lm[wr].x - lm[e].x,
                    lm[wr].y - lm[e].y,
                )
                return shoulder_wrist, elbow_wrist, vis

            left_arm = _arm_ext(_L_SHOULDER, _L_ELBOW, _L_WRIST)
            right_arm = _arm_ext(_R_SHOULDER, _R_ELBOW, _R_WRIST)
            if not left_arm and not right_arm:
                continue

            if not right_arm or (left_arm and left_arm[0] >= right_arm[0]):
                elbow, wrist, idx = _L_ELBOW, _L_WRIST, _L_INDEX
                ext_score, forearm_score, arm_vis = left_arm
            else:
                elbow, wrist, idx = _R_ELBOW, _R_WRIST, _R_INDEX
                ext_score, forearm_score, arm_vis = right_arm

            wu, wv = lm[wrist].x * w, lm[wrist].y * h
            index_vis = _landmark_visibility(lm[idx])

            # Prefer fingertip direction, fall back to forearm direction.
            if index_vis > 0.3:
                pu, pv = lm[idx].x * w, lm[idx].y * h
                conf = min(_landmark_visibility(lm[wrist]), index_vis)
            else:
                du = lm[wrist].x - lm[elbow].x
                dv = lm[wrist].y - lm[elbow].y
                if math.hypot(du, dv) < 1e-6:
                    continue
                pu, pv = wu + du * w * 0.75, wv + dv * h * 0.75
                conf = min(
                    _landmark_visibility(lm[wrist]),
                    _landmark_visibility(lm[elbow]),
                )

            length = math.hypot(pu - wu, pv - wv)
            if length < 1.0:
                continue

            score = (ext_score + forearm_score) * arm_vis * max(conf, 0.0)
            candidate = {
                'wu': wu,
                'wv': wv,
                'pu': pu,
                'pv': pv,
                'conf': conf,
            }
            if score > best_score:
                best = candidate
                best_score = score

        if best is None:
            return None

        # 3D origin
        origin_3d = None
        cam = self._pixel_to_cam3d(best['wu'], best['wv'])
        if cam is not None:
            origin_3d = self._cam3d_to_odom(cam[0], cam[1], cam[2])

        # 3D direction
        dir_3d = None
        if origin_3d is not None:
            cam_p = self._pixel_to_cam3d(best['pu'], best['pv'])
            if cam_p is not None:
                p3d = self._cam3d_to_odom(cam_p[0], cam_p[1], cam_p[2])
                if p3d is not None:
                    dx = p3d[0] - origin_3d[0]
                    dy = p3d[1] - origin_3d[1]
                    dz = p3d[2] - origin_3d[2]
                    norm = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if norm > 1e-6:
                        dir_3d = (dx / norm, dy / norm, dz / norm)

        return {
            'wu': best['wu'],
            'wv': best['wv'],
            'pu': best['pu'],
            'pv': best['pv'],
            'conf': best['conf'],
            'origin_3d': origin_3d,
            'dir_3d': dir_3d,
        }

    # ------------------------------------------------------------------
    # Main detection loop
    # ------------------------------------------------------------------
    def run_detection(self):
        if not self.detection_enabled or self.rgb_image is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_detect_time).nanoseconds / 1e9
        if dt < self.detect_interval_sec:
            return
        self.last_detect_time = now

        bgr = self.rgb_image.copy()
        stamp = now.to_msg()

        # --- YOLO detection ---
        obj_array = DetectedObjectArray()
        obj_array.header.stamp = stamp
        obj_array.header.frame_id = 'odom'

        if self.model is not None:
            preds = self.model.predict(bgr, conf=0.25, verbose=False)
            boxes = preds[0].boxes

            for box in boxes:
                cls_id = int(box.cls[0].item())
                cls_name = self.coco_names.get(cls_id, f'class_{cls_id}')
                conf = float(box.conf[0].item())
                cx, cy, bw, bh = [int(x) for x in box.xywh[0].tolist()]
                x1, y1, x2, y2 = [int(x) for x in box.xyxy[0].tolist()]

                obj = DetectedObject()
                obj.class_name = cls_name
                obj.confidence = conf
                obj.bbox_cx = cx
                obj.bbox_cy = cy
                obj.bbox_w = bw
                obj.bbox_h = bh
                obj.has_3d_position = False
                obj.depth_mm = 0.0

                cam = self._pixel_to_cam3d(cx, cy)
                if cam is not None:
                    odom_pos = self._cam3d_to_odom(cam[0], cam[1], cam[2])
                    if odom_pos is not None:
                        obj.position_3d = Point(
                            x=odom_pos[0], y=odom_pos[1], z=odom_pos[2])
                        obj.has_3d_position = True
                        obj.depth_mm = cam[3]

                obj_array.objects.append(obj)

                color = (255, 0, 0) if cls_name == 'person' else (0, 255, 0)
                cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)
                cv2.putText(bgr, f'{cls_name} {conf:.2f}', (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        self.objects_pub.publish(obj_array)

        # --- Avatar pointing ---
        pt_msg = PointingDirection()
        pt_msg.header.stamp = stamp
        pt_msg.header.frame_id = 'odom'
        pt_msg.is_valid = False
        pt_msg.confidence = 0.0

        info = self._estimate_pointing(bgr)
        if info is not None:
            pt_msg.wrist_pixel_x = float(info['wu'])
            pt_msg.wrist_pixel_y = float(info['wv'])
            pt_msg.point_pixel_x = float(info['pu'])
            pt_msg.point_pixel_y = float(info['pv'])
            pt_msg.confidence = float(info['conf'])
            pt_msg.is_valid = True

            if info['origin_3d'] is not None:
                pt_msg.origin = Point(
                    x=info['origin_3d'][0],
                    y=info['origin_3d'][1],
                    z=info['origin_3d'][2])
            if info['dir_3d'] is not None:
                pt_msg.direction = Vector3(
                    x=info['dir_3d'][0],
                    y=info['dir_3d'][1],
                    z=info['dir_3d'][2])

            # Debug arrow on image
            ox, oy = int(info['wu']), int(info['wv'])
            ex = int(info['pu'] + (info['pu'] - info['wu']) * 2)
            ey = int(info['pv'] + (info['pv'] - info['wv']) * 2)
            cv2.arrowedLine(bgr, (ox, oy), (ex, ey), (0, 0, 255), 3)
            cv2.circle(bgr, (ox, oy), 6, (0, 0, 255), -1)

        self.pointing_pub.publish(pt_msg)

        # Debug image
        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CleanupDetectionNode()
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

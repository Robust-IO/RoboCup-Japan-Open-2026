#!/usr/bin/env python3
"""
Head perception node for Interactive Cleanup.

Responsibilities:
  1. Estimate Avatar pointing from the head RGB-D stream.
  2. Publish coarse Avatar observations for centering logic.
  3. Publish graspable-object candidates with 3D positions.
"""

import os
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge

import tf2_ros
from tf_transformations import quaternion_matrix

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
    AvatarObservation,
    PointingDirection,
    SceneObject,
    SceneObjectArray,
)

from perception_common import (
    CAMERA_INFO_TOPIC,
    DEPTH_CAM_TOPIC,
    HEAD_ACTIVE_MODES,
    HEAD_CAM_TOPIC,
    find_pose_model_path,
    find_yolo_model_path,
    describe_pose_backend_unavailable,
    landmark_visibility,
    load_yolo_class,
    select_pose_backend,
)
from perception_visualization import build_head_marker_array
from pointing_utils import (
    arm_straightness,
    expand_person_crop,
    normalize_vector,
    pixel_to_camera_ray,
    project_pointing_pixel,
    remap_crop_landmark,
    select_pointing_endpoint,
    smooth_pointing_yaw,
)


# MediaPipe landmark indices
_L_SHOULDER, _L_ELBOW, _L_WRIST, _L_INDEX = 11, 13, 15, 19
_R_SHOULDER, _R_ELBOW, _R_WRIST, _R_INDEX = 12, 14, 16, 20

_MIN_POINTING_VISIBILITY = 0.45
_MIN_POINTING_CONFIDENCE = 0.35
_MIN_FOREARM_LENGTH_PX = 24.0
_MIN_ARM_STRAIGHTNESS = 0.55
_POINTING_SIDE_SCORE_MARGIN = 1.15
_POINTING_STATE_TIMEOUT_SEC = 1.5


class HeadPerceptionNode(Node):
    def __init__(self):
        super().__init__('head_perception_node')

        self.rgb_image = None
        self.depth_image = None
        self.bridge = CvBridge()

        self.fx = self.fy = self.cx_cam = self.cy_cam = None
        self.img_w = 640
        self.img_h = 480

        self.current_mode = 'IDLE'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, HEAD_CAM_TOPIC, self._rgb_cb, sensor_qos)
        self.create_subscription(Image, DEPTH_CAM_TOPIC, self._depth_cb, sensor_qos)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._info_cb, 10)
        self.create_subscription(String, '/cleanup_perception/mode', self._mode_cb, 10)

        self.avatar_pub = self.create_publisher(
            AvatarObservation, '/cleanup_perception/head/avatar', 10)
        self.objects_pub = self.create_publisher(
            SceneObjectArray, '/cleanup_perception/head/objects', 10)
        self.pointing_pub = self.create_publisher(
            PointingDirection, '/cleanup_perception/head/pointing', 10)
        self.debug_pub = self.create_publisher(
            Image, '/cleanup_perception/debug/head_image', 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/cleanup_perception/debug/head_markers', 10)

        self.model = None
        self.class_names = {}
        yolo_class, yolo_error = load_yolo_class()
        if yolo_class is None:
            self.get_logger().warn(
                f'{yolo_error}. Head object detection will remain disabled.')
        else:
            try:
                model_path = find_yolo_model_path()
                self.model = yolo_class(model_path)
                self.class_names = self.model.names
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            except Exception as exc:
                self.get_logger().error(
                    f'Failed to initialize YOLO model: {exc}. '
                    'Head object detection will remain disabled.')

        self.pose_detector = None
        self.pose_timestamp_ms = 0
        self.pose_backend = 'disabled'
        pose_model_path = find_pose_model_path()
        self.pose_backend = select_pose_backend(
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
                f'{describe_pose_backend_unavailable(HAS_MEDIAPIPE, HAS_MEDIAPIPE_TASKS, pose_model_path, HAS_MEDIAPIPE_SOLUTIONS)} '
                '— pointing estimation disabled')

        self.last_detect_time = self.get_clock().now()
        self.detect_interval_sec = 0.2
        self._has_display = bool(os.environ.get('DISPLAY', ''))
        self.last_pointing_yaw = None
        self.last_pointing_side = None
        self.last_pointing_time_ns = None

        self.get_logger().info('HeadPerceptionNode ready')

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

    def _rgb_cb(self, msg: Image):
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.rgb_image = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
            self.img_h, self.img_w = self.rgb_image.shape[:2]
        except Exception as exc:
            self.get_logger().error(f'RGB error: {exc}')

    def _depth_cb(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'Depth error: {exc}')

    def _info_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx_cam = msg.k[2]
            self.cy_cam = msg.k[5]
            self.get_logger().info(
                f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx_cam:.1f} cy={self.cy_cam:.1f}')

    def _mode_cb(self, msg: String):
        self.current_mode = msg.data.strip() or 'IDLE'

    def _is_active(self):
        return self.current_mode in HEAD_ACTIVE_MODES

    def _pixel_to_cam3d(self, u, v):
        if self.depth_image is None or self.fx is None:
            return None

        h, w = self.depth_image.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None

        roi = self.depth_image[
            max(0, vi - 3):min(h, vi + 4),
            max(0, ui - 3):min(w, ui + 4)].astype(float)
        valid = roi[roi > 0]
        if len(valid) == 0:
            return None

        raw = float(np.median(valid))
        depth = raw / 1000.0 if raw > 100 else raw
        x = (u - self.cx_cam) * depth / self.fx
        y = (v - self.cy_cam) * depth / self.fy
        return (x, y, depth, raw)

    def _cam3d_to_odom(self, x, y, z):
        transform = self._lookup_camera_to_odom_transform()
        if transform is None:
            return None

        pt = transform @ np.array([x, y, z, 1.0])
        return (float(pt[0]), float(pt[1]), float(pt[2]))

    def _lookup_camera_to_odom_transform(self):
        for frame in (
            'head_rgbd_sensor_rgb_frame',
            'head_rgbd_sensor_link',
            'rgbd_sensor_rgb_frame',
        ):
            try:
                ts = self.tf_buffer.lookup_transform(
                    'odom', frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                t = ts.transform.translation
                r = ts.transform.rotation
                mat = quaternion_matrix([r.x, r.y, r.z, r.w])
                mat[0:3, 3] = [t.x, t.y, t.z]
                return mat
            except Exception:
                continue
        return None

    def _estimate_origin(self, *pixels):
        for u, v in pixels:
            cam = self._pixel_to_cam3d(u, v)
            if cam is None:
                continue
            origin = self._cam3d_to_odom(cam[0], cam[1], cam[2])
            if origin is not None:
                return {
                    'cam': (float(cam[0]), float(cam[1]), float(cam[2])),
                    'odom': origin,
                    'raw_depth': float(cam[3]),
                }
        return None

    def _estimate_pointing(self, bgr, best_person=None):
        if self.pose_detector is None:
            return None

        pose_bgr = bgr
        crop_rect = None
        if best_person is not None:
            crop_rect = expand_person_crop(
                best_person['cx'],
                best_person['cy'],
                best_person['w'],
                best_person['h'],
                bgr.shape[1],
                bgr.shape[0],
            )
            x1, y1, x2, y2 = crop_rect
            if x2 - x1 >= 32 and y2 - y1 >= 32:
                pose_bgr = bgr[y1:y2, x1:x2]
            else:
                crop_rect = None

        rgb = np.ascontiguousarray(cv2.cvtColor(pose_bgr, cv2.COLOR_BGR2RGB))
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

        frame_h, frame_w = bgr.shape[:2]
        candidates = []

        for lm in landmark_sets:
            def _pixel(idx):
                if crop_rect is not None:
                    return remap_crop_landmark(lm[idx].x, lm[idx].y, crop_rect)
                return (lm[idx].x * frame_w, lm[idx].y * frame_h)

            def _arm_candidate(side, s_idx, e_idx, w_idx, i_idx):
                shoulder_vis = landmark_visibility(lm[s_idx])
                elbow_vis = landmark_visibility(lm[e_idx])
                wrist_vis = landmark_visibility(lm[w_idx])
                vis = min(shoulder_vis, elbow_vis, wrist_vis)
                if vis < _MIN_POINTING_VISIBILITY:
                    return None

                shoulder = _pixel(s_idx)
                elbow = _pixel(e_idx)
                wrist = _pixel(w_idx)
                forearm_len = math.hypot(wrist[0] - elbow[0], wrist[1] - elbow[1])
                if forearm_len < _MIN_FOREARM_LENGTH_PX:
                    return None

                straightness = arm_straightness(shoulder, elbow, wrist)
                if straightness < _MIN_ARM_STRAIGHTNESS:
                    return None

                index_tip = None
                index_vis = landmark_visibility(lm[i_idx])
                if index_vis >= _MIN_POINTING_VISIBILITY:
                    index_tip = _pixel(i_idx)

                pu, pv, projected_straightness = project_pointing_pixel(
                    shoulder, elbow, wrist, index_tip=index_tip)
                reach_len = math.hypot(wrist[0] - shoulder[0], wrist[1] - shoulder[1])
                conf = min(wrist_vis, max(index_vis, elbow_vis))
                conf *= 0.5 + 0.5 * max(straightness, projected_straightness)
                conf = max(0.0, min(1.0, conf))
                score = conf * max(forearm_len, 1.0) * (
                    0.5 + 0.5 * reach_len / max(frame_w, frame_h))
                return {
                    'side': side,
                    'shoulder': shoulder,
                    'elbow': elbow,
                    'wrist': wrist,
                    'wu': wrist[0],
                    'wv': wrist[1],
                    'pu': pu,
                    'pv': pv,
                    'conf': conf,
                    'score': score,
                }

            for candidate in (
                _arm_candidate('left', _L_SHOULDER, _L_ELBOW, _L_WRIST, _L_INDEX),
                _arm_candidate('right', _R_SHOULDER, _R_ELBOW, _R_WRIST, _R_INDEX),
            ):
                if candidate is not None:
                    candidates.append(candidate)

        if not candidates:
            return None

        candidates.sort(key=lambda item: item['score'], reverse=True)
        best = candidates[0]
        if len(candidates) > 1:
            second = candidates[1]
            if second['score'] > 1e-6 and best['score'] < second['score'] * _POINTING_SIDE_SCORE_MARGIN:
                sticky_candidate = next(
                    (candidate for candidate in candidates
                     if candidate['side'] == self.last_pointing_side),
                    None)
                if sticky_candidate is None:
                    return None
                best = dict(sticky_candidate)
                best['conf'] *= 0.9

        origin = self._estimate_origin(
            best['wrist'],
            best['elbow'],
            best['shoulder'],
        )
        origin_cam = None if origin is None else origin['cam']
        origin_3d = None if origin is None else origin['odom']

        dir_3d = None
        smoothed_yaw = None
        endpoint_mode = 'invalid'
        if origin_cam is not None and origin_3d is not None and self.fx is not None and self.fy is not None:
            ray_cam = pixel_to_camera_ray(
                best['pu'], best['pv'], self.fx, self.fy, self.cx_cam, self.cy_cam)
            point_cam = self._pixel_to_cam3d(best['pu'], best['pv'])
            endpoint_cam, endpoint_mode = select_pointing_endpoint(
                origin_cam,
                point_cam,
                ray_cam,
            )
            if endpoint_cam is not None:
                endpoint_odom = self._cam3d_to_odom(
                    endpoint_cam[0], endpoint_cam[1], endpoint_cam[2])
            else:
                endpoint_odom = None
            if endpoint_odom is not None:
                raw_dir = normalize_vector(
                    endpoint_odom[0] - origin_3d[0],
                    endpoint_odom[1] - origin_3d[1],
                    endpoint_odom[2] - origin_3d[2],
                )
            else:
                raw_dir = None
            if raw_dir is not None:
                previous_yaw = self.last_pointing_yaw
                if self.last_pointing_time_ns is not None:
                    age_sec = (
                        self.get_clock().now().nanoseconds - self.last_pointing_time_ns) / 1e9
                    if age_sec > _POINTING_STATE_TIMEOUT_SEC:
                        previous_yaw = None
                accepted, smoothed_yaw = smooth_pointing_yaw(
                    previous_yaw,
                    math.atan2(raw_dir[1], raw_dir[0]),
                    best['conf'],
                )
                if not accepted:
                    return None
                smoothed_dir = normalize_vector(
                    math.cos(smoothed_yaw),
                    math.sin(smoothed_yaw),
                    raw_dir[2],
                )
                if smoothed_dir is not None:
                    dir_3d = smoothed_dir
                    self.last_pointing_yaw = smoothed_yaw
                    self.last_pointing_side = best['side']
                    self.last_pointing_time_ns = self.get_clock().now().nanoseconds

        return {
            'wu': best['wu'],
            'wv': best['wv'],
            'pu': best['pu'],
            'pv': best['pv'],
            'conf': best['conf'],
            'origin_3d': origin_3d,
            'dir_3d': dir_3d,
            'crop_rect': crop_rect,
            'side': best['side'],
            'body_yaw': smoothed_yaw,
            'endpoint_mode': endpoint_mode,
        }

    def run_detection(self):
        if not self._is_active() or self.rgb_image is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_detect_time).nanoseconds / 1e9
        if dt < self.detect_interval_sec:
            return
        self.last_detect_time = now

        frame_bgr = self.rgb_image.copy()
        debug_bgr = frame_bgr.copy()
        stamp = now.to_msg()

        scene_msg = SceneObjectArray()
        scene_msg.header.stamp = stamp
        scene_msg.header.frame_id = 'odom'
        scene_msg.source_camera = 'head'

        avatar_msg = AvatarObservation()
        avatar_msg.header.stamp = stamp
        avatar_msg.header.frame_id = 'odom'
        avatar_msg.is_valid = False

        pointing_msg = PointingDirection()
        pointing_msg.header.stamp = stamp
        pointing_msg.header.frame_id = 'odom'
        pointing_msg.is_valid = False
        pointing_msg.confidence = 0.0

        best_person = None
        if self.model is not None:
            preds = self.model.predict(frame_bgr, conf=0.25, verbose=False)
            boxes = preds[0].boxes
            for box in boxes:
                cls_id = int(box.cls[0].item())
                cls_name = self.class_names.get(cls_id, f'class_{cls_id}')
                conf = float(box.conf[0].item())
                cx, cy, bw, bh = [int(x) for x in box.xywh[0].tolist()]
                x1, y1, x2, y2 = [int(x) for x in box.xyxy[0].tolist()]

                if cls_name == 'person':
                    area = bw * bh
                    if best_person is None or area > best_person['area']:
                        best_person = {
                            'cx': cx,
                            'cy': cy,
                            'w': bw,
                            'h': bh,
                            'conf': conf,
                            'area': area,
                        }
                    color = (255, 0, 0)
                else:
                    obj = SceneObject()
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
                            obj.position = Point(
                                x=odom_pos[0], y=odom_pos[1], z=odom_pos[2])
                            obj.has_3d_position = True
                            obj.depth_mm = cam[3]
                    scene_msg.objects.append(obj)
                    color = (0, 255, 0)

                cv2.rectangle(debug_bgr, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    debug_bgr, f'{cls_name} {conf:.2f}', (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        info = self._estimate_pointing(frame_bgr, best_person=best_person)
        if info is not None:
            pointing_msg.wrist_pixel_x = float(info['wu'])
            pointing_msg.wrist_pixel_y = float(info['wv'])
            pointing_msg.point_pixel_x = float(info['pu'])
            pointing_msg.point_pixel_y = float(info['pv'])
            pointing_msg.confidence = float(info['conf'])

            if info['origin_3d'] is not None and info['dir_3d'] is not None and info['conf'] >= _MIN_POINTING_CONFIDENCE:
                pointing_msg.is_valid = True
                pointing_msg.origin = Point(
                    x=info['origin_3d'][0],
                    y=info['origin_3d'][1],
                    z=info['origin_3d'][2])
                avatar_msg.origin_odom = pointing_msg.origin
                pointing_msg.direction = Vector3(
                    x=info['dir_3d'][0],
                    y=info['dir_3d'][1],
                    z=info['dir_3d'][2])
                avatar_msg.body_yaw = float(
                    info['body_yaw'] if info['body_yaw'] is not None
                    else math.atan2(info['dir_3d'][1], info['dir_3d'][0]))

            ox, oy = int(info['wu']), int(info['wv'])
            ex = int(info['pu'] + (info['pu'] - info['wu']) * 2)
            ey = int(info['pv'] + (info['pv'] - info['wv']) * 2)
            cv2.arrowedLine(debug_bgr, (ox, oy), (ex, ey), (0, 0, 255), 3)
            cv2.circle(debug_bgr, (ox, oy), 6, (0, 0, 255), -1)
            if info['crop_rect'] is not None:
                x1, y1, x2, y2 = info['crop_rect']
                cv2.rectangle(debug_bgr, (x1, y1), (x2, y2), (255, 255, 0), 1)
            cv2.putText(
                debug_bgr,
                f"point {info['side']} {info['endpoint_mode']} conf={info['conf']:.2f}",
                (ox, max(18, oy - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
            )

        if best_person is not None:
            avatar_msg.is_valid = True
            avatar_msg.bbox_cx = best_person['cx']
            avatar_msg.bbox_cy = best_person['cy']
            avatar_msg.bbox_w = best_person['w']
            avatar_msg.bbox_h = best_person['h']
            avatar_msg.center_error_x = float(
                (best_person['cx'] - self.img_w * 0.5) / max(self.img_w * 0.5, 1.0))
            avatar_msg.confidence = best_person['conf']
        elif info is not None:
            avatar_msg.is_valid = True
            avatar_msg.bbox_cx = int(info['wu'])
            avatar_msg.bbox_cy = int(info['wv'])
            avatar_msg.bbox_w = 0
            avatar_msg.bbox_h = 0
            avatar_msg.center_error_x = float(
                (info['wu'] - self.img_w * 0.5) / max(self.img_w * 0.5, 1.0))
            avatar_msg.confidence = float(info['conf'])

        self.avatar_pub.publish(avatar_msg)
        self.objects_pub.publish(scene_msg)
        self.pointing_pub.publish(pointing_msg)
        self.marker_pub.publish(
            build_head_marker_array(avatar_msg, scene_msg, pointing_msg))

        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_bgr, encoding='bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = HeadPerceptionNode()
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

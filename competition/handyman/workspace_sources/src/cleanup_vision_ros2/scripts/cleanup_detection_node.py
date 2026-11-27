#!/usr/bin/env python3
"""
cleanup_detection_node.py — Interactive Cleanup Vision Node (ROS 2 Humble)

Provides two capabilities for the Interactive Cleanup competition:
  1. Object detection via YOLO (COCO pretrained or custom weights)
     with depth-based 3D localization.
  2. Avatar pointing direction estimation via MediaPipe PoseLandmarker and
     HandLandmarker.

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

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

try:
    import mediapipe as mp
    from mediapipe.tasks.python import vision as mp_vision
    from mediapipe.tasks.python.core.base_options import BaseOptions
    from mediapipe.tasks.python.vision.core.vision_task_running_mode import (
        VisionTaskRunningMode,
    )
    HAS_MEDIAPIPE_TASKS = True
except (ImportError, AttributeError):
    mp = None
    mp_vision = None
    BaseOptions = None
    VisionTaskRunningMode = None
    HAS_MEDIAPIPE_TASKS = False

from cleanup_vision_ros2.msg import (
    DetectedObject,
    DetectedObjectArray,
    PointingDirection,
)

# MediaPipe landmark indices
_L_SHOULDER, _L_ELBOW, _L_WRIST, _L_INDEX = 11, 13, 15, 19
_R_SHOULDER, _R_ELBOW, _R_WRIST, _R_INDEX = 12, 14, 16, 20

# MediaPipe HandLandmarker indices.  The PIP→tip vector follows the actual
# index finger, whereas PoseLandmarker only provides a coarse hand landmark.
_HAND_WRIST, _INDEX_PIP, _INDEX_TIP = 0, 6, 8

HEAD_CAM_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_raw'
DEPTH_CAM_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_raw'
CAMERA_INFO_TOPIC = '/hsrb/head_rgbd_sensor/rgb/camera_info'


def _source_models_dir(package_name: str) -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.normpath(os.path.join(script_dir, '..', '..'))
    return os.path.join(src_root, package_name, 'models')


def _share_models_dir(package_name: str):
    try:
        return os.path.join(get_package_share_directory(package_name), 'models')
    except Exception:
        return None


def _unique_dirs(*dirs):
    seen = set()
    ordered = []
    for directory in dirs:
        if not directory:
            continue
        norm = os.path.normpath(directory)
        if norm in seen:
            continue
        seen.add(norm)
        ordered.append(norm)
    return ordered


def _find_yolo_model_path() -> str:
    """Locate YOLO weights in cleanup_vision_ros2 first, then vision_ros2."""
    model_dirs = _unique_dirs(
        _share_models_dir('cleanup_vision_ros2'),
        _source_models_dir('cleanup_vision_ros2'),
        _share_models_dir('vision_ros2'),
        _source_models_dir('vision_ros2'),
    )

    for name in ('cleanup_model.pt', 'last.pt', 'yolo12n.pt'):
        for model_dir in model_dirs:
            path = os.path.join(model_dir, name)
            if os.path.isfile(path):
                return path

    preferred_dir = model_dirs[0] if model_dirs else _source_models_dir(
        'cleanup_vision_ros2')
    os.makedirs(preferred_dir, exist_ok=True)
    return os.path.join(preferred_dir, 'yolo12n.pt')


def _find_pose_model_path():
    """Locate a local Pose Landmarker task bundle. No auto-download fallback."""
    model_dirs = _unique_dirs(
        _share_models_dir('cleanup_vision_ros2'),
        _source_models_dir('cleanup_vision_ros2'),
    )
    model_names = (
        'pose_landmarker.task',
        'pose_landmarker_full.task',
        'pose_landmarker_lite.task',
        'pose_landmarker_heavy.task',
    )

    for name in model_names:
        for model_dir in model_dirs:
            path = os.path.join(model_dir, name)
            if os.path.isfile(path):
                return path
    return None


def _find_hand_model_path():
    """Locate a local Hand Landmarker task bundle. No auto-download fallback."""
    model_dirs = _unique_dirs(
        _share_models_dir('cleanup_vision_ros2'),
        _source_models_dir('cleanup_vision_ros2'),
    )
    for name in ('hand_landmarker.task', 'hand_landmarker_full.task'):
        for model_dir in model_dirs:
            path = os.path.join(model_dir, name)
            if os.path.isfile(path):
                return path
    return None


def _landmark_visibility(landmark) -> float:
    visibility = getattr(landmark, 'visibility', None)
    return float(visibility) if visibility is not None else 0.0


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
        try:
            model_path = _find_yolo_model_path()
            self.model = YOLO(model_path)
            self.coco_names = self.model.names
            self.get_logger().info(f'YOLO model loaded: {model_path}')
        except Exception as exc:
            self.get_logger().error(
                f'Failed to initialize YOLO model: {exc}. '
                'Object detection will remain disabled.')

        # MediaPipe PoseLandmarker
        self.pose_detector = None
        self.hand_detector = None
        self.pose_timestamp_ms = 0
        self.last_pointing_status_log_ns = 0
        pose_model_path = _find_pose_model_path()
        if HAS_MEDIAPIPE_TASKS and pose_model_path:
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
                self.get_logger().warn(
                    f'PoseLandmarker initialization failed: {exc}. '
                    'Pointing estimation disabled.')
        elif not HAS_MEDIAPIPE_TASKS:
            self.get_logger().warn(
                'MediaPipe Tasks unavailable — pointing estimation disabled')
        else:
            self.get_logger().warn(
                'PoseLandmarker model asset not found in '
                'cleanup_vision_ros2/models/ — pointing estimation disabled')

        # MediaPipe HandLandmarker refines the coarse pose index landmark to
        # the actual index-finger PIP→tip direction.
        hand_model_path = _find_hand_model_path()
        if HAS_MEDIAPIPE_TASKS and hand_model_path:
            try:
                hand_options = mp_vision.HandLandmarkerOptions(
                    base_options=BaseOptions(model_asset_path=hand_model_path),
                    # Each frame uses a Pose-guided crop whose coordinates can
                    # jump, so independent IMAGE inference is more reliable
                    # than carrying a full-frame video tracking ROI.
                    running_mode=VisionTaskRunningMode.IMAGE,
                    num_hands=2,
                    min_hand_detection_confidence=0.3,
                    min_hand_presence_confidence=0.3,
                    min_tracking_confidence=0.3,
                )
                self.hand_detector = mp_vision.HandLandmarker.create_from_options(
                    hand_options)
                self.get_logger().info(
                    f'MediaPipe HandLandmarker initialized: {hand_model_path}')
            except Exception as exc:
                self.get_logger().warn(
                    f'HandLandmarker initialization failed: {exc}. '
                    'Using coarse PoseLandmarker pointing.')
        elif HAS_MEDIAPIPE_TASKS:
            self.get_logger().warn(
                'HandLandmarker model asset not found in '
                'cleanup_vision_ros2/models/ — using coarse pose pointing')

        self._has_display = bool(os.environ.get('DISPLAY', ''))
        self.last_detect_time = self.get_clock().now()
        self.detect_interval_sec = 0.2  # 5 Hz

        self.get_logger().info('CleanupDetectionNode ready')

    def destroy_node(self):
        for detector_name in ('pose_detector', 'hand_detector'):
            detector = getattr(self, detector_name, None)
            if detector is not None:
                try:
                    detector.close()
                except Exception:
                    pass
                setattr(self, detector_name, None)
        return super().destroy_node()

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
        if self.detection_enabled == msg.data:
            return
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

    def _bbox_foreground_to_cam3d(self, x1, y1, x2, y2):
        """Estimate an object's visible front surface from its bounding box.

        A tiny patch at the box centre often sees the floor or wall through a
        concave/irregular object.  That places the reported target behind the
        real object and makes the base drive through it.  Use the inner part
        of the box, identify the nearest robust depth cluster, and project the
        centroid of that cluster instead.  Keeping the visible front surface
        (rather than inventing an object thickness) is intentionally
        conservative for grasp approach.
        """
        if self.depth_image is None or self.fx is None:
            return None

        h, w = self.depth_image.shape[:2]
        x1 = int(np.clip(x1, 0, w - 1))
        x2 = int(np.clip(x2, x1 + 1, w))
        y1 = int(np.clip(y1, 0, h - 1))
        y2 = int(np.clip(y2, y1 + 1, h))

        # Exclude loose YOLO-box margins, which are dominated by background.
        inset_x = max(1, int((x2 - x1) * 0.12))
        inset_y = max(1, int((y2 - y1) * 0.12))
        ix1, ix2 = x1 + inset_x, x2 - inset_x
        iy1, iy2 = y1 + inset_y, y2 - inset_y
        if ix2 <= ix1 or iy2 <= iy1:
            return self._pixel_to_cam3d((x1 + x2) * 0.5,
                                        (y1 + y2) * 0.5)

        roi_raw = self.depth_image[iy1:iy2, ix1:ix2].astype(float)
        valid_mask = np.isfinite(roi_raw) & (roi_raw > 0)
        if np.count_nonzero(valid_mask) < 6:
            return self._pixel_to_cam3d((x1 + x2) * 0.5,
                                        (y1 + y2) * 0.5)

        valid_raw = roi_raw[valid_mask]
        millimetres = float(np.median(valid_raw)) > 100.0
        roi_m = roi_raw / 1000.0 if millimetres else roi_raw
        valid_depths = roi_m[valid_mask]
        valid_depths = valid_depths[(valid_depths > 0.05) &
                                    (valid_depths < 10.0)]
        if len(valid_depths) < 6:
            return self._pixel_to_cam3d((x1 + x2) * 0.5,
                                        (y1 + y2) * 0.5)

        # The 20th percentile rejects isolated near-depth noise while staying
        # on the object in boxes containing substantial background.
        near_depth = float(np.percentile(valid_depths, 20.0))
        cluster_band = max(0.035, near_depth * 0.04)
        foreground_mask = (valid_mask &
                           (roi_m >= near_depth - cluster_band) &
                           (roi_m <= near_depth + cluster_band))
        fy_idx, fx_idx = np.nonzero(foreground_mask)
        if len(fx_idx) < 4:
            return self._pixel_to_cam3d((x1 + x2) * 0.5,
                                        (y1 + y2) * 0.5)

        d = float(np.median(roi_m[foreground_mask]))
        u = float(ix1 + np.median(fx_idx))
        v = float(iy1 + np.median(fy_idx))
        x = (u - self.cx_cam) * d / self.fx
        y = (v - self.cy_cam) * d / self.fy
        raw = d * 1000.0 if millimetres else d
        return (x, y, d, raw)

    def _foreground_pixel_to_cam3d(self, u, v):
        """Unproject a thin foreground joint without averaging its background.

        Avatar fingers and forearms can cover only a few pixels. A median over
        the normal 7x7 object ROI therefore often returns the wall behind the
        limb. The near-depth percentile is a better estimate for these joints.
        """
        if self.depth_image is None or self.fx is None:
            return None

        h, w = self.depth_image.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None

        radius = 4
        roi = self.depth_image[
            max(0, vi - radius):min(h, vi + radius + 1),
            max(0, ui - radius):min(w, ui + radius + 1)].astype(float)
        valid = roi[roi > 0]
        if len(valid) == 0:
            return None

        raw = float(np.percentile(valid, 20.0))
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

    def _image_ray_to_odom(self, u, v):
        """Project an RGB pixel ray into odom without depending on depth.

        A pointing gesture constrains the target to an image line.  Using the
        camera ray through that line is much more stable than subtracting two
        nearly identical depth readings on a small fingertip.
        """
        if self.fx is None:
            return None

        camera_ray = np.array([
            (u - self.cx_cam) / self.fx,
            (v - self.cy_cam) / self.fy,
            1.0,
        ])
        camera_ray /= np.linalg.norm(camera_ray)

        for frame in ('head_rgbd_sensor_rgb_frame',
                      'head_rgbd_sensor_link',
                      'rgbd_sensor_rgb_frame'):
            try:
                ts = self.tf_buffer.lookup_transform(
                    'odom', frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                t = ts.transform.translation
                r = ts.transform.rotation
                rotation = quaternion_matrix([r.x, r.y, r.z, r.w])[:3, :3]
                odom_ray = rotation @ camera_ray
                odom_ray /= np.linalg.norm(odom_ray)
                return ((float(t.x), float(t.y), float(t.z)),
                        (float(odom_ray[0]), float(odom_ray[1]),
                         float(odom_ray[2])))
            except Exception:
                continue
        return None

    def _cam_vector_to_odom(self, direction):
        """Rotate a camera-optical-frame direction into odom."""
        vector = np.asarray(direction, dtype=float)
        norm = np.linalg.norm(vector)
        if norm < 1e-6:
            return None
        vector /= norm

        for frame in ('head_rgbd_sensor_rgb_frame',
                      'head_rgbd_sensor_link',
                      'rgbd_sensor_rgb_frame'):
            try:
                ts = self.tf_buffer.lookup_transform(
                    'odom', frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                r = ts.transform.rotation
                rotation = quaternion_matrix([r.x, r.y, r.z, r.w])[:3, :3]
                odom_vector = rotation @ vector
                odom_vector /= np.linalg.norm(odom_vector)
                return tuple(float(value) for value in odom_vector)
            except Exception:
                continue
        return None

    def _mediapipe_world_vector_to_odom(self, direction):
        """Rotate a MediaPipe world direction into odom.

        MediaPipe world landmarks use an image-aligned coordinate system in
        which negative Z points towards the camera.  ROS optical frames use
        positive Z away from the camera, so the depth component must be
        inverted before applying the camera TF rotation.
        """
        vector = np.asarray(direction, dtype=float).copy()
        if vector.shape != (3,):
            return None
        vector[2] *= -1.0
        return self._cam_vector_to_odom(vector)

    # ------------------------------------------------------------------
    # Avatar pointing estimation
    # ------------------------------------------------------------------
    def _estimate_pointing(self, bgr):
        if self.pose_detector is None:
            return None

        rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        now_ms = int(self.get_clock().now().nanoseconds / 1_000_000)
        self.pose_timestamp_ms = max(self.pose_timestamp_ms + 1, now_ms)

        try:
            results = self.pose_detector.detect_for_video(
                mp_image, self.pose_timestamp_ms)
        except Exception as exc:
            self.get_logger().warn(f'PoseLandmarker inference failed: {exc}')
            return None

        if not results.pose_landmarks:
            return None

        h, w = bgr.shape[:2]
        best = None
        best_score = -1.0
        person_center = None
        for pose_index, lm in enumerate(results.pose_landmarks):
            world_pose = None
            if pose_index < len(results.pose_world_landmarks):
                world_pose = results.pose_world_landmarks[pose_index]

            visible_torso = [
                lm[index] for index in (_L_SHOULDER, _R_SHOULDER)
                if _landmark_visibility(lm[index]) >= 0.3
            ]
            if visible_torso and person_center is None:
                person_center = (
                    sum(point.x for point in visible_torso) * w / len(visible_torso),
                    sum(point.y for point in visible_torso) * h / len(visible_torso),
                )

            def _pointing_arm(s, e, wr):
                """Return an intentionally extended arm in image or 3-D.

                An arm pointing towards/away from the camera is foreshortened
                in the image.  Requiring a long 2-D shoulder-to-wrist segment
                therefore rejects exactly the frontal poses used in some
                Interactive Cleanup layouts.  Pose-world geometry supplies
                the missing depth evidence while the vertical-ratio test
                continues to reject a naturally hanging arm.
                """
                vis = min(
                    _landmark_visibility(lm[s]),
                    _landmark_visibility(lm[e]),
                    _landmark_visibility(lm[wr]),
                )
                if vis < 0.25:
                    return None
                upper_x, upper_y = lm[e].x - lm[s].x, lm[e].y - lm[s].y
                lower_x, lower_y = lm[wr].x - lm[e].x, lm[wr].y - lm[e].y
                upper_len = math.hypot(upper_x, upper_y)
                lower_len = math.hypot(lower_x, lower_y)
                shoulder_wrist = math.hypot(
                    lm[wr].x - lm[s].x, lm[wr].y - lm[s].y)
                image_valid = False
                straightness_2d = -1.0
                if upper_len >= 0.02 and lower_len >= 0.02:
                    straightness_2d = (
                        upper_x * lower_x + upper_y * lower_y) / (
                            upper_len * lower_len)
                    horizontalness = (
                        abs(lm[wr].y - lm[s].y) /
                        max(shoulder_wrist, 1e-6))
                    image_valid = (
                        shoulder_wrist >= 0.10 and
                        straightness_2d >= 0.62 and
                        horizontalness <= 0.68)

                world_valid = False
                world_reach_ratio = 0.0
                straightness_3d = -1.0
                if world_pose is not None:
                    shoulder = np.array([
                        world_pose[s].x, world_pose[s].y, world_pose[s].z])
                    elbow_point = np.array([
                        world_pose[e].x, world_pose[e].y, world_pose[e].z])
                    wrist_point = np.array([
                        world_pose[wr].x, world_pose[wr].y, world_pose[wr].z])
                    upper_3d = elbow_point - shoulder
                    lower_3d = wrist_point - elbow_point
                    upper_3d_len = np.linalg.norm(upper_3d)
                    lower_3d_len = np.linalg.norm(lower_3d)
                    reach_3d = np.linalg.norm(wrist_point - shoulder)
                    shoulder_width = np.linalg.norm(np.array([
                        world_pose[_L_SHOULDER].x -
                        world_pose[_R_SHOULDER].x,
                        world_pose[_L_SHOULDER].y -
                        world_pose[_R_SHOULDER].y,
                        world_pose[_L_SHOULDER].z -
                        world_pose[_R_SHOULDER].z,
                    ]))
                    if upper_3d_len >= 0.08 and lower_3d_len >= 0.08:
                        straightness_3d = float(np.dot(
                            upper_3d, lower_3d) /
                            (upper_3d_len * lower_3d_len))
                        vertical_ratio = (
                            abs(wrist_point[1] - shoulder[1]) /
                            max(reach_3d, 1e-6))
                        world_reach_ratio = (
                            reach_3d / max(shoulder_width, 0.20))
                        world_valid = (
                            reach_3d >= 0.22 and
                            straightness_3d >= 0.52 and
                            vertical_ratio <= 0.82 and
                            world_reach_ratio >= 0.72)

                if not image_valid and not world_valid:
                    return None

                # The score only ranks the two arms of the same person.  Give
                # a real 3-D extension enough weight to beat a foreshortened
                # or naturally hanging arm in frontal views.
                geometry_score = max(
                    shoulder_wrist,
                    min(world_reach_ratio, 2.0) * 0.18)
                straightness = max(straightness_2d, straightness_3d, 0.0)
                return geometry_score, max(lower_len, 0.03), vis, straightness

            left_arm = _pointing_arm(_L_SHOULDER, _L_ELBOW, _L_WRIST)
            right_arm = _pointing_arm(_R_SHOULDER, _R_ELBOW, _R_WRIST)
            if not left_arm and not right_arm:
                continue

            if not right_arm or (left_arm and left_arm[0] >= right_arm[0]):
                elbow, wrist, idx = _L_ELBOW, _L_WRIST, _L_INDEX
                ext_score, forearm_score, arm_vis, straightness = left_arm
            else:
                elbow, wrist, idx = _R_ELBOW, _R_WRIST, _R_INDEX
                ext_score, forearm_score, arm_vis, straightness = right_arm

            wu, wv = lm[wrist].x * w, lm[wrist].y * h
            eu, ev = lm[elbow].x * w, lm[elbow].y * h
            # Start with the forearm as a last-resort fallback, then prefer the
            # Pose index joint when it is visible. This preserves the visible
            # hand/forearm angle even if the detailed hand model fails.
            du = lm[wrist].x - lm[elbow].x
            dv = lm[wrist].y - lm[elbow].y
            if math.hypot(du, dv) < 1e-6:
                continue
            pu, pv = wu + du * w * 1.25, wv + dv * h * 1.25
            ray_origin_u, ray_origin_v = eu, ev
            conf = min(
                _landmark_visibility(lm[wrist]),
                _landmark_visibility(lm[elbow]),
            )
            source = 'extended-arm fallback'
            index_conf = _landmark_visibility(lm[idx])
            index_u, index_v = lm[idx].x * w, lm[idx].y * h
            if (index_conf >= 0.2 and
                    math.hypot(index_u - wu, index_v - wv) >= 4.0):
                pu, pv = index_u, index_v
                conf = min(_landmark_visibility(lm[wrist]), index_conf)
                source = 'pose index joint'

            length = math.hypot(pu - wu, pv - wv)
            if length < 1.0:
                # A true arm pointing almost along the optical axis can
                # collapse to a sub-pixel segment.  Keep a minimal segment so
                # the valid 3-D pose-world direction can still be published;
                # the controller deliberately refuses to aim the head from
                # such a short and noisy 2-D arrow.
                if world_pose is None:
                    continue
                pixel_norm = max(math.hypot(du * w, dv * h), 1e-6)
                pu = wu + du * w / pixel_norm
                pv = wv + dv * h / pixel_norm

            score = ((ext_score + forearm_score) * arm_vis *
                     straightness * max(conf, 0.0))
            candidate = {
                # Unity's Laser transform is parented to the forearm and
                # LookAt() targets Index4. Mirror that long-baseline geometry.
                'wu': ray_origin_u,
                'wv': ray_origin_v,
                'pu': pu,
                'pv': pv,
                'hand_wu': wu,
                'hand_wv': wv,
                'conf': conf,
                'source': source,
                'joint_refined': source == 'pose index joint',
                'forearm_px': math.hypot(
                    (lm[wrist].x - lm[elbow].x) * w,
                    (lm[wrist].y - lm[elbow].y) * h),
            }
            if world_pose is not None:
                direction_endpoint = idx if index_conf >= 0.2 else wrist
                candidate['pose_world_direction'] = np.array([
                    world_pose[direction_endpoint].x - world_pose[elbow].x,
                    world_pose[direction_endpoint].y - world_pose[elbow].y,
                    world_pose[direction_endpoint].z - world_pose[elbow].z,
                ])
            if score > best_score:
                best = candidate
                best_score = score

        if best is None:
            if person_center is None:
                return None
            return {
                'person_center': person_center,
                'is_pointing': False,
            }

        # Pose gives the arm that is extended, but its hand/index landmarks are
        # too coarse to distinguish a pointing finger from an open hand.  Match
        # that pose wrist to MediaPipe's hand result, then use index PIP→tip as
        # the ray.  The PointingDirection message keeps the historical
        # wrist_pixel_* field names; when this refinement succeeds they mean
        # the ray origin (index PIP), rather than the anatomical wrist.
        finger_world_direction = None
        if self.hand_detector is not None:
            try:
                # Pose identifies the active wrist. Crop generously around it
                # and upscale because the synthetic hand is tiny at 640x480.
                crop_radius = int(np.clip(best['forearm_px'] * 1.15, 48, 110))
                x0 = max(0, int(best['hand_wu']) - crop_radius)
                y0 = max(0, int(best['hand_wv']) - crop_radius)
                x1 = min(w, int(best['hand_wu']) + crop_radius)
                y1 = min(h, int(best['hand_wv']) + crop_radius)
                hand_rgb = rgb[y0:y1, x0:x1]
                if hand_rgb.size == 0:
                    raise ValueError('empty hand crop')
                crop_h, crop_w = hand_rgb.shape[:2]
                scale = max(1.0, 320.0 / max(crop_h, crop_w))
                if scale > 1.0:
                    hand_rgb = cv2.resize(
                        hand_rgb, None, fx=scale, fy=scale,
                        interpolation=cv2.INTER_CUBIC)
                hand_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=np.ascontiguousarray(hand_rgb))
                hand_results = self.hand_detector.detect(hand_image)
                closest_hand = None
                closest_distance = float('inf')
                for hand_index, hand_landmarks in enumerate(
                        hand_results.hand_landmarks):
                    if len(hand_landmarks) <= _INDEX_TIP:
                        continue
                    hand_wrist = hand_landmarks[_HAND_WRIST]
                    mapped_wrist_u = x0 + hand_wrist.x * crop_w
                    mapped_wrist_v = y0 + hand_wrist.y * crop_h
                    distance = math.hypot(
                        mapped_wrist_u - best['hand_wu'],
                        mapped_wrist_v - best['hand_wv'])
                    if distance < closest_distance:
                        closest_hand = hand_index
                        closest_distance = distance

                # A pose wrist and hand wrist farther apart than one quarter
                # of the image diagonal cannot reliably belong to one arm.
                if (closest_hand is not None and
                        closest_distance <= crop_radius):
                    hand = hand_results.hand_landmarks[closest_hand]
                    pip = hand[_INDEX_PIP]
                    tip = hand[_INDEX_TIP]
                    pip_u, pip_v = x0 + pip.x * crop_w, y0 + pip.y * crop_h
                    tip_u, tip_v = x0 + tip.x * crop_w, y0 + tip.y * crop_h
                    if math.hypot(tip_u - pip_u, tip_v - pip_v) >= 8.0:
                        best['pu'], best['pv'] = tip_u, tip_v
                        best['finger_refined'] = True
                        best['joint_refined'] = True
                        best['source'] = 'index finger'
                        if closest_hand < len(hand_results.hand_world_landmarks):
                            world = hand_results.hand_world_landmarks[closest_hand]
                            world_pip = world[_INDEX_PIP]
                            world_tip = world[_INDEX_TIP]
                            finger_world_direction = np.array([
                                world_tip.x - world_pip.x,
                                world_tip.y - world_pip.y,
                                world_tip.z - world_pip.z,
                            ])
                        if closest_hand < len(hand_results.handedness):
                            categories = hand_results.handedness[closest_hand]
                            if categories:
                                best['conf'] = min(
                                    1.0, max(best['conf'],
                                             float(categories[0].score)))
            except Exception as exc:
                self.get_logger().warn(
                    f'HandLandmarker inference failed: {exc}; '
                    'using coarse PoseLandmarker pointing')

        # Prefer the same long-baseline forearm→fingertip ray used by Unity.
        origin_3d = None
        dir_3d = None
        forearm_cam = self._foreground_pixel_to_cam3d(
            best['wu'], best['wv'])
        if forearm_cam is not None:
            origin_3d = self._cam3d_to_odom(
                forearm_cam[0], forearm_cam[1], forearm_cam[2])

        if best.get('joint_refined', False):
            tip_cam = self._foreground_pixel_to_cam3d(
                best['pu'], best['pv'])
            if forearm_cam is not None and tip_cam is not None:
                depth_direction = (
                    np.array(tip_cam[:3]) - np.array(forearm_cam[:3]))
                depth_length = np.linalg.norm(depth_direction)
                if 0.05 <= depth_length <= 1.2:
                    dir_3d = self._cam_vector_to_odom(depth_direction)
        if dir_3d is None and best.get('pose_world_direction') is not None:
            dir_3d = self._mediapipe_world_vector_to_odom(
                best['pose_world_direction'])
        if dir_3d is None and finger_world_direction is not None:
            dir_3d = self._mediapipe_world_vector_to_odom(
                finger_world_direction)

        # Final fallback: use the camera bearing through the extended 2-D
        # finger/forearm line. It remains useful, but is lower precision.
        ray_u = best['pu'] + (best['pu'] - best['wu']) * 8.0
        ray_v = best['pv'] + (best['pv'] - best['wv']) * 8.0
        if origin_3d is None or dir_3d is None:
            image_ray = self._image_ray_to_odom(ray_u, ray_v)
            if image_ray is not None:
                fallback_origin, fallback_direction = image_ray
                if origin_3d is None:
                    origin_3d = fallback_origin
                if dir_3d is None:
                    dir_3d = fallback_direction

        return {
            'wu': best['wu'],
            'wv': best['wv'],
            'pu': best['pu'],
            'pv': best['pv'],
            'conf': best['conf'],
            'finger_refined': best.get('finger_refined', False),
            'source': best['source'],
            'person_center': person_center,
            'is_pointing': True,
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
        # Run pose/hand inference on the camera pixels before drawing YOLO
        # overlays, which otherwise can obscure the Avatar's fingertip.
        pointing_bgr = bgr.copy()
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

                cam = self._bbox_foreground_to_cam3d(x1, y1, x2, y2)
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
        pt_msg.person_is_valid = False
        pt_msg.confidence = 0.0

        info = self._estimate_pointing(pointing_bgr)
        if info is not None and info.get('person_center') is not None:
            pt_msg.person_pixel_x = float(info['person_center'][0])
            pt_msg.person_pixel_y = float(info['person_center'][1])
            pt_msg.person_is_valid = True

        if info is not None and info.get('is_pointing', False):
            pt_msg.wrist_pixel_x = float(info['wu'])
            pt_msg.wrist_pixel_y = float(info['wv'])
            pt_msg.point_pixel_x = float(info['pu'])
            pt_msg.point_pixel_y = float(info['pv'])
            pt_msg.confidence = float(info['conf'])
            pt_msg.is_valid = True

            now_ns = now.nanoseconds
            if now_ns - self.last_pointing_status_log_ns >= 2_000_000_000:
                source = info['source']
                ray_state = 'world ray ready' if info['dir_3d'] is not None else 'no world ray'
                self.get_logger().info(
                    f'Pointing active: {source}, {ray_state}, '
                    f'confidence={info["conf"]:.2f}')
                self.last_pointing_status_log_ns = now_ns

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

            # Red means Pose fallback; yellow means the arrow follows the
            # actual index-finger PIP→tip segment.
            ox, oy = int(info['wu']), int(info['wv'])
            ex = int(info['pu'] + (info['pu'] - info['wu']) * 2)
            ey = int(info['pv'] + (info['pv'] - info['wv']) * 2)
            arrow_color = (0, 255, 255) if info['finger_refined'] else (0, 0, 255)
            cv2.arrowedLine(bgr, (ox, oy), (ex, ey), arrow_color, 3)
            cv2.circle(bgr, (ox, oy), 6, arrow_color, -1)

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

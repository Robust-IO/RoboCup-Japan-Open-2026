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
import time
import json
from collections import deque
from rosidl_runtime_py.convert import message_to_ordereddict

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge

import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_matrix

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from pointing_utils import (
    shoulder_tracking_center,
    merge_multiscale_boxes,
    fit_projective_depth_line,
    is_extended_index_finger,
    select_synchronized_pair,
    pointing_plane_normal,
    pointing_forward_normal,
    mediapipe_to_optical,
    robust_foreground_depth,
)

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
_L_HIP, _R_HIP = 23, 24

# MediaPipe HandLandmarker indices.  The PIP→tip vector follows the actual
# index finger, whereas PoseLandmarker only provides a coarse hand landmark.
_HAND_WRIST, _INDEX_MCP, _INDEX_PIP, _INDEX_TIP = 0, 5, 6, 8

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
        self.rgb_stamp = None
        self.depth_stamp = None
        self.last_processed_rgb_stamp_ns = None
        self.last_sync_warning_ns = 0
        self.depth_frame_usable = False
        self.depth_scale = 1.0
        self.frame_camera_transform = None
        self.frame_camera_transform_resolved = False
        self.diagnostic_frames = deque(maxlen=3)
        self.diagnostic_samples = deque(maxlen=48)
        self.diagnostic_sample_time = None
        self.diagnostic_capture_index = 0
        self.diagnostic_last_wall_ns = None
        self.diagnostic_capture_dir = os.path.expanduser(
            '~/.ros/cleanup_perception/' + time.strftime('%Y%m%d-%H%M%S') +
            '-' + str(os.getpid()))
        # Keep short timestamped queues instead of pairing whichever two
        # callbacks happened to run most recently. Pose/YOLO inference blocks
        # this single-threaded node for long enough that "latest RGB + latest
        # depth" was frequently 0.3--0.7 s apart in field logs.
        self.rgb_frames = deque(maxlen=10)
        self.depth_frames = deque(maxlen=10)
        self.bridge = CvBridge()

        # Camera intrinsics (filled from CameraInfo)
        self.fx = self.fy = self.cx_cam = self.cy_cam = None
        self.img_w = 640
        self.img_h = 480

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        # Inference blocks this node's sensor executor. Give TF a separate
        # node/thread so capture-time lookups can receive transforms while
        # YOLO/MediaPipe are running (or waiting for a transform).
        # launch's process-wide __node remap belongs to the detector, not
        # this private listener. Inheriting it creates two identically named
        # nodes and unregisters the wrong rosout publisher during shutdown.
        self.tf_node = Node('cleanup_detection_tf', context=self.context,
                            use_global_arguments=False)
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self.tf_node, spin_thread=True)

        # Detection gate
        self.detection_enabled = False

        # QoS for sensor streams
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
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
        self.detection_scales = tuple(self.declare_parameter(
            'detection_scales', [640, 320, 160]).value)
        if not self.detection_scales or any(s <= 0 or s % 32 for s in self.detection_scales):
            raise ValueError('detection_scales must contain positive multiples of 32')
        self.last_object_inference_sec = 0.0
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
        self.last_detect_time = time.monotonic()
        self.detect_interval_sec = 0.2  # 5 Hz

        self.ready_pub = self.create_publisher(
            Bool, '/cleanup_vision/ready',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        # Loading weights does not initialize the inference backend. Exercise
        # the production image pyramid before allowing the competition to start.
        if self.model is None or self.pose_detector is None:
            raise RuntimeError('Required vision models unavailable; cannot declare ready')
        warmup_start = time.monotonic()
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        self._detect_object_boxes(blank)
        warmup_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=blank)
        self.pose_detector.detect_for_video(warmup_image, 0)
        if self.hand_detector is not None:
            self.hand_detector.detect(warmup_image)
        self.get_logger().info(
            f'Vision inference warmup completed in {time.monotonic()-warmup_start:.2f}s')
        self.ready_pub.publish(Bool(data=True))
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
        self.tf_listener.executor.shutdown()
        self.tf_listener.dedicated_listener_thread.join()
        self.tf_listener.unregister()
        self.tf_node.destroy_node()
        return super().destroy_node()

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------
    def _rgb_cb(self, msg: Image):
        received_steady = time.monotonic()
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.img_h, self.img_w = self.rgb_image.shape[:2]
            self.rgb_frames.append((
                self.rgb_stamp.nanoseconds, self.rgb_stamp,
                self.rgb_image, received_steady))
        except Exception as e:
            self.get_logger().error(f'RGB error: {e}')

    def _depth_cb(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            # ROS 16UC1 is millimetres; 32FC1 is metres. Units must never
            # depend on whether a particular pixel happens to exceed 100.
            self.depth_scale = 0.001 if msg.encoding.upper() == '16UC1' else 1.0
            self.depth_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.depth_frames.append((
                self.depth_stamp.nanoseconds, self.depth_stamp,
                self.depth_image, self.depth_scale))
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
        if not msg.data and self.diagnostic_frames:
            self._save_diagnostic_frames()

    def _save_diagnostic_frames(self):
        """Retain actual inputs and geometry for offline failure reproduction."""
        try:
            os.makedirs(self.diagnostic_capture_dir, exist_ok=True)
            for index, frame in enumerate(self.diagnostic_frames):
                path = os.path.join(
                    self.diagnostic_capture_dir,
                    f'phase{self.diagnostic_capture_index:02d}-frame{index}.npz')
                np.savez_compressed(path, **frame)
            for index, frame in enumerate(self.diagnostic_samples):
                path = os.path.join(
                    self.diagnostic_capture_dir,
                    f'phase{self.diagnostic_capture_index:02d}-sample{index:02d}.npz')
                np.savez_compressed(path, **frame)
            self.diagnostic_capture_index += 1
            self.get_logger().info(
                f'Perception evidence saved: {self.diagnostic_capture_dir}')
            self.diagnostic_frames.clear()
            self.diagnostic_samples.clear()
            self.diagnostic_sample_time = None
        except Exception as exc:
            self.get_logger().warn(f'Cannot save perception evidence: {exc}')

    def _select_rgbd_pair(self):
        """Return a synchronized queued pair, waiting briefly for depth.

        The main loop calls detection after every single ROS callback. If RGB
        arrives first, processing it immediately guarantees use of the
        previous depth frame. Hold that RGB for up to 0.8 s so its registered
        depth callback can enter the queue, then degrade to 2-D rather than
        stalling perception indefinitely.
        """
        if not self.rgb_frames or not self.depth_frames:
            return None
        rgb_candidates = [
            frame for frame in self.rgb_frames
            if frame[0] != self.last_processed_rgb_stamp_ns]
        if not rgb_candidates:
            return None

        pair = select_synchronized_pair(
            [frame[0] for frame in rgb_candidates],
            [frame[0] for frame in self.depth_frames])
        if pair is not None:
            rgb_index, depth_index, delta_ns = pair
            return (rgb_candidates[rgb_index], self.depth_frames[depth_index],
                    delta_ns / 1e9, True)

        newest_rgb = max(rgb_candidates, key=lambda frame: frame[0])
        if time.monotonic() - newest_rgb[3] < 0.8:
            return None
        nearest_depth = min(
            self.depth_frames,
            key=lambda frame: abs(frame[0] - newest_rgb[0]))
        delta = abs(nearest_depth[0] - newest_rgb[0]) / 1e9
        return newest_rgb, nearest_depth, delta, False

    # ------------------------------------------------------------------
    # 3-D helpers
    # ------------------------------------------------------------------
    def _pixel_to_cam3d(self, u, v):
        """Return (x, y, z, raw_depth) in camera frame, or None."""
        if (not self.depth_frame_usable or self.depth_image is None or
                self.fx is None):
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
        d = raw * self.depth_scale

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
        if (not self.depth_frame_usable or self.depth_image is None or
                self.fx is None):
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
        millimetres = self.depth_scale == 0.001
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
        if (not self.depth_frame_usable or self.depth_image is None or
                self.fx is None):
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

        d = robust_foreground_depth(valid * self.depth_scale)
        if d is None:
            return None
        raw = d / self.depth_scale
        x = (u - self.cx_cam) * d / self.fx
        y = (v - self.cy_cam) * d / self.fy
        return (x, y, d, raw)

    def _lookup_transform(self, target_frame, source_frame, capture_time=None):
        """Use capture-time TF; preserve 2-D tracking if it is unavailable."""
        self.last_tf_lookup_error = ''
        lookup_times = []
        if capture_time is not None and capture_time.nanoseconds != 0:
            lookup_times.append(capture_time)
        # A historical image must not be transformed with the current head
        # pose. This used to mix two views whenever inference lagged behind.
        if not lookup_times:
            lookup_times.append(rclpy.time.Time())
        for lookup_time in lookup_times:
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame, source_frame, lookup_time,
                    timeout=rclpy.duration.Duration(seconds=0.1))
            except tf2_ros.TransformException as exc:
                # Keep the actual reason: missing frame, disconnected tree,
                # past extrapolation and future extrapolation need different
                # repairs. Do not hide programming errors as missing TF.
                self.last_tf_lookup_error = (
                    f'{source_frame}->{target_frame}: {type(exc).__name__}: {exc}')
                if isinstance(exc, tf2_ros.ExtrapolationException):
                    stationary = self._stationary_tf_boundary(
                        target_frame, source_frame, lookup_time)
                    if stationary is not None:
                        return stationary
        return None

    def _stationary_tf_boundary(self, target, source, requested):
        """Bound a small future timestamp gap using an observed still pose.

        SIGVerse streams may arrive a few milliseconds apart. Never substitute
        a later/current pose for an old image, or use this across head motion.
        The returned transform retains its real timestamp (it is approximate).
        """
        if requested.nanoseconds == 0:
            return None
        try:
            latest = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time())
            stamp = rclpy.time.Time.from_msg(latest.header.stamp)
            gap = (requested.nanoseconds - stamp.nanoseconds) / 1e9
            if not 0.0 < gap <= 0.040:
                return None
            earlier = self.tf_buffer.lookup_transform(
                target, source, stamp - rclpy.duration.Duration(seconds=0.1))
        except tf2_ros.TransformException:
            return None
        a, b = latest.transform, earlier.transform
        translation = np.linalg.norm([
            a.translation.x-b.translation.x, a.translation.y-b.translation.y,
            a.translation.z-b.translation.z])
        qa = np.array([a.rotation.x, a.rotation.y, a.rotation.z, a.rotation.w])
        qb = np.array([b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w])
        norms = np.linalg.norm(qa) * np.linalg.norm(qb)
        if not np.isfinite(norms) or norms < 1e-9:
            return None
        angle = 2.0 * math.acos(float(np.clip(abs(np.dot(qa, qb))/norms, 0, 1)))
        if not np.isfinite(translation) or translation > 0.002 or angle > 0.005:
            return None
        self.get_logger().info(
            f'TF boundary compensation: {gap*1000:.1f}ms, '
            f'100ms motion={translation:.4f}m/{angle:.4f}rad',
            throttle_duration_sec=2.0)
        return latest

    def _camera_transform(self, capture_time):
        # Resolve once per RGB-D capture and share it across objects, arm and
        # image plane. RGB optical axes must not use a robot link frame.
        if not self.frame_camera_transform_resolved:
            self.frame_camera_transform_resolved = True
            lookup_errors = []
            for frame in ('head_rgbd_sensor_rgb_frame', 'rgbd_sensor_rgb_frame'):
                ts = self._lookup_transform('odom', frame, capture_time)
                if ts is not None:
                    self.frame_camera_transform = ts
                    break
                lookup_errors.append(self.last_tf_lookup_error)
            if self.frame_camera_transform is None:
                self.get_logger().warn(
                    'Capture-time optical TF unavailable: publishing tracking '
                    'only, not inventing odom geometry; ' + '; '.join(lookup_errors),
                    throttle_duration_sec=2.0)
        return self.frame_camera_transform

    def _cam3d_to_odom(self, x, y, z, capture_time=None):
        """Transform camera-frame point to odom frame.  Returns (x,y,z) or None."""
        # Try the proper camera optical frame first
        for ts in (self._camera_transform(capture_time),):
            if ts is not None:
                t = ts.transform.translation
                r = ts.transform.rotation
                mat = quaternion_matrix([r.x, r.y, r.z, r.w])
                mat[0:3, 3] = [t.x, t.y, t.z]
                pt = mat @ np.array([x, y, z, 1.0])
                return (float(pt[0]), float(pt[1]), float(pt[2]))

        return None

    def _cam_vector_to_odom(self, direction, capture_time=None):
        """Rotate a camera-optical-frame direction into odom."""
        vector = np.asarray(direction, dtype=float)
        norm = np.linalg.norm(vector)
        if norm < 1e-6:
            return None
        vector /= norm

        for ts in (self._camera_transform(capture_time),):
            if ts is not None:
                r = ts.transform.rotation
                rotation = quaternion_matrix([r.x, r.y, r.z, r.w])[:3, :3]
                odom_vector = rotation @ vector
                odom_vector /= np.linalg.norm(odom_vector)
                return tuple(float(value) for value in odom_vector)
        return None

    def _mediapipe_world_vector_to_odom(self, direction, capture_time=None):
        """Rotate a MediaPipe world direction into odom.

        Smaller MediaPipe Z is closer to camera, just as in ROS optical
        coordinates. Flipping Z reflects rather than rotates the skeleton.
        """
        vector = np.asarray(direction, dtype=float).copy()
        if vector.shape != (3,):
            return None
        vector = mediapipe_to_optical(vector)
        return self._cam_vector_to_odom(vector, capture_time) if vector else None

    def _fit_arm_depth_direction(self, origin_u, origin_v, end_u, end_v):
        """Fit a measured 3-D arm ray from several foreground depth samples."""
        samples = []
        for t in np.linspace(0.0, 1.0, 9):
            u = origin_u + (end_u - origin_u) * float(t)
            v = origin_v + (end_v - origin_v) * float(t)
            point = self._foreground_pixel_to_cam3d(u, v)
            if point is not None:
                samples.append((float(t), point[0], point[1], point[2]))
        return fit_projective_depth_line(samples)

    # ------------------------------------------------------------------
    # Avatar pointing estimation
    # ------------------------------------------------------------------
    def _estimate_close_hand_pointing(self, rgb, capture_time=None, image_region=None):
        """Recover a pointing ray when a nearby body is cropped from view.

        PoseLandmarker commonly loses a person whose torso does not fit in the
        image, even though the much larger hand is clear. Full-frame hand
        inference supplies both a tracking anchor and an index-finger ray in
        that case. This is a bounded fallback: an actually extended index
        finger is required, and synchronized RGB-D remains preferred.
        """
        hand_audit = {'status': 'not_run', 'hands': []}
        if hasattr(self, 'gesture_audit'):
            self.gesture_audit['close_hand'] = hand_audit
        if self.hand_detector is None:
            hand_audit['status'] = 'detector_unavailable'
            return None
        h, w = rgb.shape[:2]
        x0, y0, x1, y1 = image_region or (0, 0, w, h)
        crop_w, crop_h = x1-x0, y1-y0
        if crop_w <= 0 or crop_h <= 0:
            return None
        hand_audit['image_region'] = [x0, y0, x1, y1]
        hand_rgb = rgb[y0:y1, x0:x1]
        if image_region is not None:
            scale = max(1.0, 320.0/max(crop_w, crop_h))
            hand_rgb = cv2.resize(hand_rgb, None, fx=scale, fy=scale,
                                  interpolation=cv2.INTER_CUBIC)
        try:
            hand_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=np.ascontiguousarray(hand_rgb))
            results = self.hand_detector.detect(hand_image)
        except Exception as exc:
            self.get_logger().warn(
                f'Close-range HandLandmarker inference failed: {exc}')
            hand_audit['status'] = 'inference_error'
            return None
        hand_audit['status'] = 'detected' if results.hand_landmarks else 'no_hands'
        if not results.hand_landmarks:
            return None

        h, w = rgb.shape[:2]
        best = None
        first_center = None
        for hand_index, hand in enumerate(results.hand_landmarks):
            if len(hand) <= _INDEX_TIP:
                continue
            # Always return original camera pixels for RGB-D sampling and
            # image-plane construction, including after crop upscaling.
            pixel = lambda index: (x0 + hand[index].x * crop_w,
                                   y0 + hand[index].y * crop_h)
            wrist = pixel(_HAND_WRIST)
            mcp = pixel(_INDEX_MCP)
            pip = pixel(_INDEX_PIP)
            tip = pixel(_INDEX_TIP)
            palm_indices = (_HAND_WRIST, 5, 9, 13, 17)
            center = (
                sum(pixel(index)[0] for index in palm_indices) /
                len(palm_indices),
                sum(pixel(index)[1] for index in palm_indices) /
                len(palm_indices),
            )
            if first_center is None:
                first_center = center
            extended = is_extended_index_finger(wrist, mcp, pip, tip)
            hand_audit['hands'].append({
                'landmarks': [[float(p.x), float(p.y), float(p.z)] for p in hand],
                'index_extended': bool(extended),
            })
            if not extended:
                continue
            confidence = 0.70
            # Handedness is a left/right classification, not localization
            # or pointing accuracy. Keep the existing uncalibrated fallback
            # weight; do not promote it using a different classifier's score.
            length = math.hypot(tip[0] - pip[0], tip[1] - pip[1])
            score = confidence * length
            if best is None or score > best['score']:
                best = {
                    'score': score,
                    'confidence': confidence,
                    'center': center,
                    'wrist': wrist,
                    'mcp': mcp,
                    'pip': pip,
                    'tip': tip,
                    'hand_index': hand_index,
                }

        if best is None:
            return {
                'person_center': first_center,
                'is_pointing': False,
            }

        hand_world_direction = None
        hand_index = best['hand_index']
        if hand_index < len(results.hand_world_landmarks):
            world = results.hand_world_landmarks[hand_index]
            hand_world_direction = np.array([
                world[_INDEX_TIP].x - world[_INDEX_PIP].x,
                world[_INDEX_TIP].y - world[_INDEX_PIP].y,
                world[_INDEX_TIP].z - world[_INDEX_PIP].z,
            ])
        pose_dir_3d = None
        if hand_world_direction is not None:
            pose_dir_3d = self._mediapipe_world_vector_to_odom(
                hand_world_direction, capture_time)

        depth_fit = self._fit_arm_depth_direction(
            best['mcp'][0], best['mcp'][1],
            best['tip'][0], best['tip'][1])
        origin_3d = None
        dir_3d = None
        depth_direction_valid = False
        depth_fit_error = float('inf')
        depth_fit_samples = 0
        origin_cam = self._foreground_pixel_to_cam3d(
            best['pip'][0], best['pip'][1])
        if depth_fit is not None:
            (fit_origin, fit_direction, depth_fit_error,
             depth_fit_samples, fit_length) = depth_fit
            measured_direction = self._cam_vector_to_odom(
                fit_direction, capture_time)
            agreement = 1.0
            if measured_direction is not None and pose_dir_3d is not None:
                agreement = sum(a * b for a, b in zip(
                    measured_direction, pose_dir_3d))
            endpoint_depth = (
                fit_origin[2] + fit_direction[2] * fit_length)
            if (measured_direction is not None and
                    agreement >= math.cos(math.radians(55.0)) and
                    fit_length <= 0.25 and
                    0.20 <= fit_origin[2] <= 4.5 and
                    0.20 <= endpoint_depth <= 4.5):
                origin_cam = (*fit_origin, 0.0)
                dir_3d = measured_direction
                depth_direction_valid = True
        if origin_cam is not None:
            origin_3d = self._cam3d_to_odom(
                origin_cam[0], origin_cam[1], origin_cam[2], capture_time)
        if dir_3d is None:
            dir_3d = pose_dir_3d

        return {
            'hand_wrist': best['wrist'],
            'wu': best['pip'][0],
            'wv': best['pip'][1],
            'pu': best['tip'][0],
            'pv': best['tip'][1],
            'geometry_source': 'hand',
            'conf': best['confidence'],
            'finger_refined': True,
            'source': 'wrist-crop hand fallback' if image_region is not None
                      else 'close-range hand fallback',
            'person_center': best['center'],
            'is_pointing': True,
            'origin_3d': origin_3d,
            'dir_3d': dir_3d,
            'pose_dir_3d': pose_dir_3d,
            'depth_direction_valid': depth_direction_valid,
            'depth_fit_error': depth_fit_error,
            'depth_fit_samples': depth_fit_samples,
        }

    def _associate_pointing_hand(self, hand, results, width, height):
        """Recover the forearm of a recognized hand, including bent arms.

        Arm extension is one way to detect pointing intent, not a prerequisite
        for using visible elbow/wrist landmarks after finger recognition.
        All landmarks here belong to this same image; no cached body is used.
        """
        wrist_pixel = hand.get('hand_wrist')
        if wrist_pixel is None or not all(math.isfinite(v) for v in wrist_pixel):
            return None
        best = None
        best_distance = float('inf')
        for pose_index, lm in enumerate(results.pose_landmarks):
            for elbow, wrist, index in ((_L_ELBOW, _L_WRIST, _L_INDEX),
                                        (_R_ELBOW, _R_WRIST, _R_INDEX)):
                if min(_landmark_visibility(lm[elbow]),
                       _landmark_visibility(lm[wrist])) < 0.25:
                    continue
                eu, ev = lm[elbow].x*width, lm[elbow].y*height
                wu, wv = lm[wrist].x*width, lm[wrist].y*height
                if not all(math.isfinite(v) for v in (eu, ev, wu, wv)):
                    continue
                if not (0 <= eu < width and 0 <= ev < height):
                    continue
                distance = math.hypot(wu-wrist_pixel[0], wv-wrist_pixel[1])
                # Same association radius as the existing wrist crop. This
                # associates a hand with its arm; it is not a pointing gate.
                if distance > 64.0 or distance >= best_distance:
                    continue
                candidate = {
                    'wu': eu, 'wv': ev, 'pu': hand['pu'], 'pv': hand['pv'],
                    'hand_wu': wu, 'hand_wv': wv,
                    'conf': min(hand['conf'], _landmark_visibility(lm[elbow]),
                                _landmark_visibility(lm[wrist])),
                    'source': 'finger-confirmed forearm',
                    'joint_refined': True, 'finger_refined': True,
                    'forearm_px': math.hypot(wu-eu, wv-ev),
                    'person_center': hand.get('person_center'),
                }
                if pose_index < len(results.pose_world_landmarks):
                    world = results.pose_world_landmarks[pose_index]
                    endpoint = index if _landmark_visibility(lm[index]) >= 0.2 else wrist
                    candidate['pose_world_direction'] = np.array([
                        getattr(world[endpoint], axis)-getattr(world[elbow], axis)
                        for axis in ('x', 'y', 'z')])
                best, best_distance = candidate, distance
        return best

    def _estimate_pointing(self, bgr, capture_time=None):
        self.gesture_audit = {'pose_status': 'not_run', 'poses': [], 'arms': []}
        if self.pose_detector is None:
            self.gesture_audit['pose_status'] = 'detector_unavailable'
            return None

        rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        # MediaPipe needs elapsed video time, not a wall clock that can jump
        # during host/WSL clock synchronization. Capture stamps remain intact
        # for RGB-D pairing and TF lookup.
        now_ms = time.monotonic_ns() // 1_000_000
        self.pose_timestamp_ms = max(self.pose_timestamp_ms + 1, now_ms)

        try:
            results = self.pose_detector.detect_for_video(
                mp_image, self.pose_timestamp_ms)
        except Exception as exc:
            self.get_logger().warn(f'PoseLandmarker inference failed: {exc}')
            self.gesture_audit['pose_status'] = 'inference_error'
            return None

        h, w = bgr.shape[:2]
        self.gesture_audit['pose_status'] = 'detected' if results.pose_landmarks else 'no_pose'
        if not results.pose_landmarks:
            return self._estimate_close_hand_pointing(rgb, capture_time)

        best = None
        best_score = -1.0
        person_center = None
        for pose_index, lm in enumerate(results.pose_landmarks):
            world_pose = None
            if pose_index < len(results.pose_world_landmarks):
                world_pose = results.pose_world_landmarks[pose_index]
            self.gesture_audit['poses'].append({
                'image': [[float(p.x), float(p.y), float(p.z),
                           float(_landmark_visibility(p))] for p in lm],
                'world': [[float(p.x), float(p.y), float(p.z)] for p in world_pose]
                         if world_pose is not None else None,
                'image_size': [w, h],
            })

            # Restore shoulder-centred framing from the earlier controller.
            # Mixing hips into a visible shoulder centre aimed the camera
            # below the pointing arm. A single visible shoulder is enough;
            # torso/finite-shoulder fallbacks preserve person tracking.
            pose_person_center = shoulder_tracking_center(
                [[p.x, p.y, p.z, _landmark_visibility(p)] for p in lm], w, h)
            if pose_person_center is None:
                left_shoulder = lm[_L_SHOULDER]
                right_shoulder = lm[_R_SHOULDER]
                if all(math.isfinite(value) for value in (
                        left_shoulder.x, left_shoulder.y,
                        right_shoulder.x, right_shoulder.y)):
                    pose_person_center = (
                        0.5 * (left_shoulder.x + right_shoulder.x) * w,
                        0.5 * (left_shoulder.y + right_shoulder.y) * h,
                    )
            if pose_person_center is not None and person_center is None:
                person_center = pose_person_center

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
                audit = {'pose_index': pose_index, 'shoulder_index': s,
                         'visibility': float(vis), 'accepted': False,
                         'reason': 'low_visibility'}
                self.gesture_audit['arms'].append(audit)
                if vis < 0.25:
                    return None
                upper_x, upper_y = lm[e].x - lm[s].x, lm[e].y - lm[s].y
                lower_x, lower_y = lm[wr].x - lm[e].x, lm[wr].y - lm[e].y
                upper_len = math.hypot(upper_x, upper_y)
                lower_len = math.hypot(lower_x, lower_y)
                shoulder_wrist = math.hypot(
                    lm[wr].x - lm[s].x, lm[wr].y - lm[s].y)
                image_valid = False
                horizontalness = None
                vertical_ratio = None
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

                audit.update({
                    'image_valid': bool(image_valid), 'world_valid': bool(world_valid),
                    'image_reach': float(shoulder_wrist),
                    'image_straightness': float(straightness_2d),
                    'image_vertical_ratio': horizontalness,
                    'world_straightness': float(straightness_3d),
                    'world_vertical_ratio': float(vertical_ratio) if vertical_ratio is not None else None,
                    'world_reach_ratio': float(world_reach_ratio),
                    'accepted': bool(image_valid or world_valid),
                    'reason': 'accepted' if image_valid or world_valid else 'extension_checks_failed',
                })
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
                'person_center': pose_person_center,
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
            close_hand = self._estimate_close_hand_pointing(rgb, capture_time)
            if close_hand is None or not close_hand.get('is_pointing', False):
                # Arm-extension failure must not prevent independent finger
                # recognition. At most two visible wrist crops, with the same
                # extended-index/depth checks as the full-frame fallback.
                attempts = [self.gesture_audit.get('close_hand', {})]
                crops = []
                for lm in results.pose_landmarks:
                    for wrist in (_L_WRIST, _R_WRIST):
                        p = lm[wrist]
                        if (_landmark_visibility(p) < 0.5 or
                                not 0.0 <= p.x <= 1.0 or not 0.0 <= p.y <= 1.0):
                            continue
                        u, v = int(p.x*w), int(p.y*h)
                        if any(math.hypot(u-a, v-b) < 48 for a, b in crops):
                            continue
                        crops.append((u, v))
                for u, v in crops[:2]:
                    candidate = self._estimate_close_hand_pointing(
                        rgb, capture_time,
                        (max(0, u-64), max(0, v-64), min(w, u+64), min(h, v+64)))
                    attempts.append(self.gesture_audit.get('close_hand', {}))
                    if candidate is not None and candidate.get('is_pointing', False):
                        close_hand = candidate
                        break
                self.gesture_audit['hand_fallback_attempts'] = attempts
            if close_hand is not None and close_hand.get('is_pointing', False):
                if person_center is not None:
                    close_hand['person_center'] = person_center
                best = self._associate_pointing_hand(close_hand, results, w, h)
                if best is None:
                    return close_hand
            if best is None:
                if person_center is None:
                    return close_hand
                return {
                    'person_center': person_center,
                    'is_pointing': False,
                }

        if best.get('person_center') is not None:
            person_center = best['person_center']

        # Refine the endpoint of the elbow-to-index baseline. The historical
        # wrist_pixel_* fields contain the elbow in this full-arm path;
        # only the independent hand-only path uses index PIP as its origin.
        finger_world_direction = None
        if self.hand_detector is not None and not best.get('finger_refined', False):
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
            except Exception as exc:
                self.get_logger().warn(
                    f'HandLandmarker inference failed: {exc}; '
                    'using coarse PoseLandmarker pointing')

        # Prefer the same long-baseline forearm→fingertip ray used by Unity.
        origin_3d = None
        dir_3d = None
        pose_dir_3d = None
        if best.get('pose_world_direction') is not None:
            pose_dir_3d = self._mediapipe_world_vector_to_odom(
                best['pose_world_direction'], capture_time)

        historical_arm_ray = self._historical_arm_ray(
            best, pose_dir_3d, capture_time)
        depth_fit = None
        depth_direction_valid = False
        depth_fit_error = float('inf')
        depth_fit_samples = 0
        if best.get('joint_refined', False):
            depth_fit = self._fit_arm_depth_direction(
                best['wu'], best['wv'], best['pu'], best['pv'])
        forearm_cam = self._foreground_pixel_to_cam3d(best['wu'], best['wv'])
        if depth_fit is not None:
            (fit_origin, fit_direction, depth_fit_error,
             depth_fit_samples, fit_length) = depth_fit
            measured_odom_direction = self._cam_vector_to_odom(
                fit_direction, capture_time)
            # Sensor depth and pose-world estimates are independent.  A large
            # disagreement means the sampled pixels likely belong to the
            # background, so do not label that line as measured geometry.
            agreement = 1.0
            if measured_odom_direction is not None and pose_dir_3d is not None:
                agreement = sum(a * b for a, b in zip(
                    measured_odom_direction, pose_dir_3d))
            fit_endpoint_depth = (
                fit_origin[2] + fit_direction[2] * fit_length)
            plausible_depth_range = (
                0.45 <= fit_origin[2] <= 6.0 and
                0.45 <= fit_endpoint_depth <= 6.0)
            torso_depth_consistent = True
            if person_center is not None:
                torso_cam = self._foreground_pixel_to_cam3d(
                    person_center[0], person_center[1])
                if torso_cam is not None:
                    torso_depth_consistent = (
                        abs(fit_origin[2] - torso_cam[2]) <= 0.65)
            if (measured_odom_direction is not None and
                    agreement >= math.cos(math.radians(50.0)) and
                    fit_length <= 0.75 and
                    plausible_depth_range and torso_depth_consistent):
                forearm_cam = (*fit_origin, 0.0)
                dir_3d = measured_odom_direction
                depth_direction_valid = True
        if forearm_cam is not None:
            origin_3d = self._cam3d_to_odom(
                forearm_cam[0], forearm_cam[1], forearm_cam[2], capture_time)
        if dir_3d is None and best.get('pose_world_direction') is not None:
            dir_3d = pose_dir_3d
        if dir_3d is None and finger_world_direction is not None:
            dir_3d = self._mediapipe_world_vector_to_odom(
                finger_world_direction, capture_time)

        # A missing spatial direction stays missing. The image line is
        # published separately as a plane; it cannot manufacture arm depth.

        return {
            'historical_arm_ray': historical_arm_ray,
            'wu': best['wu'],
            'wv': best['wv'],
            'pu': best['pu'],
            'pv': best['pv'],
            'geometry_source': 'arm',
            'conf': best['conf'],
            'finger_refined': best.get('finger_refined', False),
            'source': best['source'],
            'person_center': person_center,
            'is_pointing': True,
            'origin_3d': origin_3d,
            'dir_3d': dir_3d,
            'pose_dir_3d': pose_dir_3d,
            'depth_direction_valid': depth_direction_valid,
            'depth_fit_error': depth_fit_error,
            'depth_fit_samples': depth_fit_samples,
        }

    def _historical_arm_ray(self, best, pose_direction, capture_time):
        """d902b25 endpoint/pose geometry for an already accepted full arm.

        Keep capture-time TF and require an actual localized origin. The old
        camera-bearing substitution is deliberately not a hand-origin ray.
        """
        forearm = self._foreground_pixel_to_cam3d(best['wu'], best['wv'])
        if forearm is None:
            return None
        origin = self._cam3d_to_odom(*forearm[:3], capture_time)
        direction = None
        if best.get('joint_refined', False):
            tip = self._foreground_pixel_to_cam3d(best['pu'], best['pv'])
            if tip is not None:
                delta = np.asarray(tip[:3]) - np.asarray(forearm[:3])
                if 0.05 <= np.linalg.norm(delta) <= 1.2:
                    direction = self._cam_vector_to_odom(delta, capture_time)
        if direction is None:
            direction = pose_direction
        if origin is None or direction is None:
            return None
        if not np.all(np.isfinite([*origin, *direction])):
            return None
        norm = float(np.linalg.norm(direction))
        if norm < 1e-6:
            return None
        return origin, tuple(float(v / norm) for v in direction)

    # ------------------------------------------------------------------
    # Main detection loop
    # ------------------------------------------------------------------
    def _detect_object_boxes(self, bgr):
        start = time.monotonic()
        boxes = []
        for size in self.detection_scales:
            boxes.extend(self.model.predict(
                bgr, imgsz=size, conf=0.25, iou=0.7, verbose=False)[0].boxes)
        rows = [box.xyxy[0].tolist()+[float(box.conf[0]), int(box.cls[0])]
                for box in boxes]
        merged = [boxes[i] for i in merge_multiscale_boxes(rows)]
        self.last_object_inference_sec = time.monotonic()-start
        return merged

    def run_detection(self):
        if not self.detection_enabled:
            return

        # Rate limiting must not stall perception after a wall-clock rewind.
        now_monotonic = time.monotonic()
        dt = now_monotonic - self.last_detect_time
        if dt < self.detect_interval_sec:
            return
        self.last_detect_time = now_monotonic

        selected = self._select_rgbd_pair()
        if selected is None:
            return
        now = self.get_clock().now()
        rgb_frame, depth_frame, sync_delta, synchronized = selected
        rgb_stamp_ns, capture_time, rgb_image, _ = rgb_frame
        _, depth_time, depth_image, self.depth_scale = depth_frame
        self.frame_camera_transform = None
        self.frame_camera_transform_resolved = False
        self.rgb_image = rgb_image
        self.rgb_stamp = capture_time
        self.depth_image = depth_image
        self.depth_stamp = depth_time
        # Person pose and image-space tracking do not require depth. Never
        # suppress those observations merely because SIGVerse publishes the
        # registered depth stream later than RGB. Depth geometry is enabled
        # only for a reasonably close pair and explicitly marked as such.
        self.depth_frame_usable = synchronized
        if not self.depth_frame_usable:
            if now.nanoseconds - self.last_sync_warning_ns >= 2_000_000_000:
                self.get_logger().warn(
                    f'RGB-D delta {sync_delta:.3f}s: publishing person/2-D '
                    'gesture only; depth geometry disabled for this frame')
                self.last_sync_warning_ns = now.nanoseconds
        self.last_processed_rgb_stamp_ns = rgb_stamp_ns
        # Drop frames at or before the one just consumed; later callbacks can
        # still pair newer RGB with their own registered depth image.
        self.rgb_frames = deque(
            (frame for frame in self.rgb_frames if frame[0] > rgb_stamp_ns),
            maxlen=10)

        bgr = rgb_image.copy()
        # Run pose/hand inference on the camera pixels before drawing YOLO
        # overlays, which otherwise can obscure the Avatar's fingertip.
        pointing_bgr = bgr.copy()
        stamp = capture_time.to_msg()

        # --- YOLO detection ---
        obj_array = DetectedObjectArray()
        obj_array.received_steady_ns = int(rgb_frame[3] * 1_000_000_000)
        obj_array.header.stamp = stamp
        obj_array.header.frame_id = 'odom'

        diagnostic_objects = []
        if self.model is not None:
            # Keep native-scale small objects and recover near objects via
            # the same image pyramid, independent of task, class or state.
            boxes = self._detect_object_boxes(bgr)

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
                diagnostic_objects.append({
                    'class_name': cls_name, 'bbox_xyxy': [x1, y1, x2, y2],
                    'camera_xyz': list(cam[:3]) if cam is not None else None,
                    'sample_pixel': [self.fx*cam[0]/cam[2]+self.cx_cam,
                                     self.fy*cam[1]/cam[2]+self.cy_cam]
                        if cam is not None and cam[2] > 0 else None,
                })
                if cam is not None:
                    odom_pos = self._cam3d_to_odom(
                        cam[0], cam[1], cam[2], capture_time)
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
        pt_msg.received_steady_ns = int(rgb_frame[3] * 1_000_000_000)
        pt_msg.header.stamp = stamp
        pt_msg.header.frame_id = 'odom'
        pt_msg.is_valid = False
        pt_msg.person_is_valid = False
        pt_msg.pose_direction_valid = False
        pt_msg.depth_direction_valid = False
        pt_msg.depth_direction_fit_error = 0.0
        pt_msg.depth_direction_samples = 0
        pt_msg.confidence = 0.0

        info = self._estimate_pointing(pointing_bgr, capture_time)
        if info is not None and info.get('person_center') is not None:
            pt_msg.person_pixel_x = float(info['person_center'][0])
            pt_msg.person_pixel_y = float(info['person_center'][1])
            pt_msg.person_is_valid = True

        if info is not None and info.get('is_pointing', False):
            pt_msg.wrist_pixel_x = float(info['wu'])
            pt_msg.wrist_pixel_y = float(info['wv'])
            pt_msg.point_pixel_x = float(info['pu'])
            pt_msg.point_pixel_y = float(info['pv'])
            pt_msg.geometry_source = info.get('geometry_source', 'unknown')
            pt_msg.confidence = float(info['conf'])
            pt_msg.is_valid = True
            arm_ray = info.get('historical_arm_ray')
            if arm_ray is not None:
                origin, direction = arm_ray
                pt_msg.arm_ray_valid = True
                pt_msg.arm_ray_origin = Point(x=origin[0], y=origin[1], z=origin[2])
                pt_msg.arm_ray_direction = Vector3(
                    x=direction[0], y=direction[1], z=direction[2])
            normal = pointing_plane_normal(
                (info['wu'], info['wv']), (info['pu'], info['pv']),
                self.fx, self.fy, self.cx_cam, self.cy_cam)
            camera = self._camera_transform(capture_time)
            if normal is not None and camera is not None:
                forward = pointing_forward_normal(
                    (info['wu'], info['wv']), (info['pu'], info['pv']),
                    self.fx, self.fy, self.cx_cam, self.cy_cam)
                world_normal = self._cam_vector_to_odom(normal, capture_time)
                world_forward = (self._cam_vector_to_odom(forward, capture_time)
                                 if forward is not None else None)
                camera_forward = self._cam_vector_to_odom((0, 0, 1), capture_time)
                if world_normal is not None and world_forward is not None:
                    pt_msg.plane_origin = Point(
                        x=camera.transform.translation.x,
                        y=camera.transform.translation.y,
                        z=camera.transform.translation.z)
                    pt_msg.plane_normal = Vector3(
                        x=world_normal[0], y=world_normal[1], z=world_normal[2])
                    pt_msg.plane_forward = Vector3(
                        x=world_forward[0], y=world_forward[1], z=world_forward[2])
                    pt_msg.camera_forward = Vector3(
                        x=camera_forward[0], y=camera_forward[1], z=camera_forward[2])
                    pt_msg.plane_valid = True

            now_ns = now.nanoseconds
            if now_ns - self.last_pointing_status_log_ns >= 2_000_000_000:
                source = info['source']
                ray_state = 'world ray ready' if info['dir_3d'] is not None else 'no world ray'
                depth_state = (
                    f'depth_fit(n={info["depth_fit_samples"]}, '
                    f'rmse={info["depth_fit_error"]:.3f}m)'
                    if info['depth_direction_valid'] else 'pose/image fallback')
                image_segment = math.hypot(
                    info['pu'] - info['wu'], info['pv'] - info['wv'])
                self.get_logger().info(
                    f'Pointing active: {source}, {ray_state}, {depth_state}, '
                    f'image_segment={image_segment:.1f}px, '
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
            pt_msg.depth_direction_valid = bool(info['depth_direction_valid'])
            if math.isfinite(info['depth_fit_error']):
                pt_msg.depth_direction_fit_error = float(info['depth_fit_error'])
            pt_msg.depth_direction_samples = min(
                255, int(info['depth_fit_samples']))
            if info['pose_dir_3d'] is not None:
                pt_msg.pose_direction = Vector3(
                    x=info['pose_dir_3d'][0],
                    y=info['pose_dir_3d'][1],
                    z=info['pose_dir_3d'][2])
                pt_msg.pose_direction_valid = True

            # Red means Pose fallback; yellow means the arrow follows the
            # actual index-finger PIP→tip segment.
            ox, oy = int(info['wu']), int(info['wv'])
            ex = int(info['pu'] + (info['pu'] - info['wu']) * 2)
            ey = int(info['pv'] + (info['pv'] - info['wv']) * 2)
            arrow_color = (0, 255, 255) if info['finger_refined'] else (0, 0, 255)
            cv2.arrowedLine(bgr, (ox, oy), (ex, ey), arrow_color, 3)
            cv2.circle(bgr, (ox, oy), 6, arrow_color, -1)

        self.pointing_pub.publish(pt_msg)
        camera = self.frame_camera_transform
        camera_matrix = np.full((4, 4), np.nan)
        if camera is not None:
            rotation = camera.transform.rotation
            translation = camera.transform.translation
            camera_matrix = quaternion_matrix(
                [rotation.x, rotation.y, rotation.z, rotation.w])
            camera_matrix[:3, 3] = [translation.x, translation.y, translation.z]
        self.diagnostic_frames.append({
            'rgb_bgr': rgb_image.copy(),
            'depth': depth_image.copy(),
            'depth_scale': self.depth_scale,
            'rgb_stamp_ns': rgb_stamp_ns,
            'depth_stamp_ns': depth_time.nanoseconds,
            'intrinsics': np.array([self.fx, self.fy, self.cx_cam, self.cy_cam]),
            'camera_to_odom': camera_matrix,
            'objects_json': json.dumps(message_to_ordereddict(obj_array)),
            'pointing_json': json.dumps(message_to_ordereddict(pt_msg)),
        })
        # Preserve the pointing period, not just the final lowered arm. Share
        # the immutable frame snapshot; bounded sampling uses no disk IO here.
        sample_time = time.monotonic()
        if (self.diagnostic_sample_time is None or
                sample_time-self.diagnostic_sample_time >= 1.0):
            self.diagnostic_samples.append(self.diagnostic_frames[-1])
            self.diagnostic_sample_time = sample_time
        # Small per-frame metadata covers the entire observation.
        try:
            wall_ns = time.time_ns()
            previous = self.diagnostic_last_wall_ns
            self.diagnostic_last_wall_ns = wall_ns
            record = {
                'phase': self.diagnostic_capture_index,
                'wall_ns': wall_ns, 'monotonic_ns': time.monotonic_ns(),
                'received_steady_ns': pt_msg.received_steady_ns,
                'wall_reversed': previous is not None and wall_ns < previous,
                'rgb_stamp_ns': rgb_stamp_ns,
                'depth_stamp_ns': depth_time.nanoseconds,
                'depth_usable': bool(self.depth_frame_usable),
                'detection_scales': self.detection_scales,
                'object_inference_sec': self.last_object_inference_sec,
                'rgb_callback_to_result_sec': time.monotonic()-rgb_frame[3],
                'stamp_to_result_sec': (wall_ns-rgb_stamp_ns)/1e9,
                'geometry_source': info.get('geometry_source', 'none') if info else 'none',
                'camera_to_odom': camera_matrix.tolist() if camera is not None else None,
                'tf_stamp': message_to_ordereddict(camera.header.stamp) if camera else None,
                'depth_samples': diagnostic_objects,
                'objects': message_to_ordereddict(obj_array),
                'pointing': message_to_ordereddict(pt_msg),
                'gesture_audit': self.gesture_audit,
            }
            os.makedirs(self.diagnostic_capture_dir, exist_ok=True)
            with open(os.path.join(self.diagnostic_capture_dir, 'observations.jsonl'),
                      'a', encoding='utf-8') as stream:
                stream.write(json.dumps(record) + '\n')
        except Exception as exc:
            self.get_logger().warning(f'Observation trace write failed: {exc}')

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

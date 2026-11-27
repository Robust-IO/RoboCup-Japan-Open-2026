#!/usr/bin/env python3

import importlib
import os

from ament_index_python.packages import get_package_share_directory


HEAD_CAM_TOPIC = '/hsrb/head_rgbd_sensor/rgb/image_raw'
DEPTH_CAM_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_raw'
CAMERA_INFO_TOPIC = '/hsrb/head_rgbd_sensor/rgb/camera_info'

HAND_CAM_TOPIC = '/hsrb/hand_camera/image_raw'
HAND_CAMERA_INFO_TOPIC = '/hsrb/hand_camera/camera_info'

HEAD_ACTIVE_MODES = {
    'TRACK_AVATAR',
    'RESOLVE_PICK',
    'RESOLVE_DEST',
}

HAND_ACTIVE_MODES = {
    'HAND_APPROACH',
    'HAND_VERIFY',
}


def source_models_dir(package_name: str) -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.normpath(os.path.join(script_dir, '..', '..'))
    return os.path.join(src_root, package_name, 'models')


def share_models_dir(package_name: str):
    try:
        return os.path.join(get_package_share_directory(package_name), 'models')
    except Exception:
        return None


def unique_dirs(*dirs):
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


def find_yolo_model_path() -> str:
    """Locate YOLO weights in cleanup_vision_ros2 first, then vision_ros2."""
    model_dirs = unique_dirs(
        share_models_dir('cleanup_vision_ros2'),
        source_models_dir('cleanup_vision_ros2'),
        share_models_dir('vision_ros2'),
        source_models_dir('vision_ros2'),
    )

    for name in ('cleanup_model.pt', 'last.pt', 'yolo12n.pt'):
        for model_dir in model_dirs:
            path = os.path.join(model_dir, name)
            if os.path.isfile(path):
                return path

    preferred_dir = model_dirs[0] if model_dirs else source_models_dir(
        'cleanup_vision_ros2')
    os.makedirs(preferred_dir, exist_ok=True)
    return os.path.join(preferred_dir, 'yolo12n.pt')


def find_pose_model_path():
    """Locate a local Pose Landmarker task bundle. No auto-download fallback."""
    model_dirs = unique_dirs(
        share_models_dir('cleanup_vision_ros2'),
        source_models_dir('cleanup_vision_ros2'),
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


def landmark_visibility(landmark) -> float:
    visibility = getattr(landmark, 'visibility', None)
    return float(visibility) if visibility is not None else 0.0


def load_yolo_class():
    try:
        ultralytics = importlib.import_module('ultralytics')
    except ImportError as exc:
        return None, f'ultralytics import failed: {exc}'

    yolo_class = getattr(ultralytics, 'YOLO', None)
    if yolo_class is None:
        return None, 'ultralytics imported but YOLO class is unavailable'

    return yolo_class, ''


def select_pose_backend(
    has_mediapipe_tasks: bool,
    pose_model_path,
    has_mediapipe_solutions: bool,
) -> str:
    if has_mediapipe_tasks and pose_model_path:
        return 'tasks'
    if has_mediapipe_solutions:
        return 'solutions'
    return 'disabled'


def describe_pose_backend_unavailable(
    has_mediapipe: bool,
    has_mediapipe_tasks: bool,
    pose_model_path,
    has_mediapipe_solutions: bool,
) -> str:
    if has_mediapipe_tasks and not pose_model_path:
        message = 'PoseLandmarker model asset not found in cleanup_vision_ros2/models/'
        if not has_mediapipe_solutions:
            message += ' and MediaPipe Pose fallback unavailable'
        return message

    if not has_mediapipe and not has_mediapipe_tasks:
        return 'MediaPipe unavailable'

    if has_mediapipe and not has_mediapipe_solutions:
        return 'MediaPipe Pose fallback unavailable'

    return 'Pose estimation unavailable'

#!/usr/bin/env python3

import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_vector(x, y, z=0.0):
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-6:
        return None
    return (x / norm, y / norm, z / norm)


def expand_person_crop(
    cx,
    cy,
    width,
    height,
    image_width,
    image_height,
    width_scale=1.5,
    height_scale=1.6,
):
    half_width = max(12.0, width * width_scale * 0.5)
    half_height = max(12.0, height * height_scale * 0.5)
    x1 = int(max(0.0, math.floor(cx - half_width)))
    y1 = int(max(0.0, math.floor(cy - half_height)))
    x2 = int(min(float(image_width), math.ceil(cx + half_width)))
    y2 = int(min(float(image_height), math.ceil(cy + half_height)))
    return (x1, y1, x2, y2)


def remap_crop_landmark(landmark_x, landmark_y, crop_rect):
    x1, y1, x2, y2 = crop_rect
    crop_width = max(1, x2 - x1)
    crop_height = max(1, y2 - y1)
    return (
        x1 + landmark_x * crop_width,
        y1 + landmark_y * crop_height,
    )


def arm_straightness(shoulder, elbow, wrist):
    upper_x = elbow[0] - shoulder[0]
    upper_y = elbow[1] - shoulder[1]
    forearm_x = wrist[0] - elbow[0]
    forearm_y = wrist[1] - elbow[1]
    upper_norm = math.hypot(upper_x, upper_y)
    forearm_norm = math.hypot(forearm_x, forearm_y)
    if upper_norm < 1e-6 or forearm_norm < 1e-6:
        return 0.0
    cosine = (upper_x * forearm_x + upper_y * forearm_y) / (
        upper_norm * forearm_norm)
    return clamp((cosine + 1.0) * 0.5, 0.0, 1.0)


def project_pointing_pixel(
    shoulder,
    elbow,
    wrist,
    index_tip=None,
    min_extension_px=36.0,
    extension_scale=1.15,
    finger_weight=0.35,
):
    forearm_x = wrist[0] - elbow[0]
    forearm_y = wrist[1] - elbow[1]
    forearm_norm = math.hypot(forearm_x, forearm_y)
    if forearm_norm < 1e-6:
        return wrist[0], wrist[1], 0.0

    direction = (forearm_x / forearm_norm, forearm_y / forearm_norm)
    if index_tip is not None:
        finger_x = index_tip[0] - wrist[0]
        finger_y = index_tip[1] - wrist[1]
        finger_norm = math.hypot(finger_x, finger_y)
        if finger_norm >= 1e-6:
            finger_dir = (finger_x / finger_norm, finger_y / finger_norm)
            if direction[0] * finger_dir[0] + direction[1] * finger_dir[1] > 0.0:
                blend_x = direction[0] * (1.0 - finger_weight) + finger_dir[0] * finger_weight
                blend_y = direction[1] * (1.0 - finger_weight) + finger_dir[1] * finger_weight
                blended = normalize_vector(blend_x, blend_y)
                if blended is not None:
                    direction = (blended[0], blended[1])

    extension = max(min_extension_px, forearm_norm * extension_scale)
    straightness = arm_straightness(shoulder, elbow, wrist)
    return (
        wrist[0] + direction[0] * extension,
        wrist[1] + direction[1] * extension,
        straightness,
    )


def pixel_to_camera_ray(u, v, fx, fy, cx, cy):
    return normalize_vector(
        (u - cx) / max(fx, 1e-6),
        (v - cy) / max(fy, 1e-6),
        1.0,
    )


def project_ray_to_depth(point_ray, target_depth_z):
    if point_ray is None or target_depth_z <= 1e-6:
        return None
    if abs(point_ray[2]) < 1e-6:
        return None

    scale = target_depth_z / point_ray[2]
    if scale <= 0.0:
        return None
    return (
        point_ray[0] * scale,
        point_ray[1] * scale,
        point_ray[2] * scale,
    )


def is_depth_consistent_with_origin(
    origin_cam,
    point_cam,
    max_abs_delta=0.15,
    max_rel_delta=0.08,
):
    if origin_cam is None or point_cam is None:
        return False

    origin_depth = float(origin_cam[2])
    point_depth = float(point_cam[2])
    if origin_depth <= 1e-6 or point_depth <= 1e-6:
        return False

    tolerance = max(max_abs_delta, origin_depth * max_rel_delta)
    return abs(point_depth - origin_depth) <= tolerance


def select_pointing_endpoint(
    origin_cam,
    point_cam,
    point_ray,
    max_abs_delta=0.15,
    max_rel_delta=0.08,
):
    if origin_cam is None:
        return None, 'invalid'

    if is_depth_consistent_with_origin(
        origin_cam,
        point_cam,
        max_abs_delta=max_abs_delta,
        max_rel_delta=max_rel_delta,
    ):
        return (float(point_cam[0]), float(point_cam[1]), float(point_cam[2])), 'depth'

    fallback = project_ray_to_depth(point_ray, float(origin_cam[2]))
    if fallback is not None:
        return fallback, 'wrist_depth'

    if point_cam is not None:
        return (float(point_cam[0]), float(point_cam[1]), float(point_cam[2])), 'depth_unchecked'

    return None, 'invalid'


def smooth_pointing_yaw(
    previous_yaw,
    raw_yaw,
    confidence,
    alpha=0.35,
    max_low_conf_jump=0.55,
):
    raw_yaw = normalize_angle(raw_yaw)
    if previous_yaw is None:
        return True, raw_yaw

    delta = normalize_angle(raw_yaw - previous_yaw)
    if confidence < 0.5 and abs(delta) > max_low_conf_jump:
        return False, previous_yaw

    effective_alpha = clamp(alpha * (0.5 + 0.5 * confidence), 0.05, 1.0)
    return True, normalize_angle(previous_yaw + delta * effective_alpha)

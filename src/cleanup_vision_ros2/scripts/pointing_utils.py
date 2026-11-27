#!/usr/bin/env python3

import math


def person_image_height_fraction(landmarks):
    """Estimate visible head-to-foot extent, including cropped image edges."""
    def visible_y(indices):
        return [landmarks[i][1] for i in indices if i < len(landmarks)
                and landmarks[i][3] >= 0.5 and math.isfinite(landmarks[i][1])]
    if not visible_y(range(11, 27)):
        return None
    head = visible_y(range(0, 11))
    feet = visible_y(range(27, 33))
    top = max(0.0, min(1.0, min(head))) if head else 0.0
    bottom = max(0.0, min(1.0, max(feet))) if feet else 1.0
    return max(0.0, bottom - top)


def shoulder_tracking_center(landmarks, width, height):
    """Keep upper-body pointing in view; hips are only a detection fallback."""
    def visible(indices, threshold):
        return [landmarks[i] for i in indices
                if landmarks[i][3] >= threshold and
                all(math.isfinite(v) for v in landmarks[i][:2])]
    points = visible((11, 12), 0.3)
    if not points:
        points = visible((11, 12, 23, 24), 0.1)
    if not points:
        return None
    return (sum(p[0] for p in points)*width/len(points),
            sum(p[1] for p in points)*height/len(points))


def pointing_plane_normal(start, end, fx, fy, cx, cy):
    """Normal of the camera plane containing an observed image line."""
    if math.hypot(end[0] - start[0], end[1] - start[1]) < 5.0:
        return None
    a = ((start[0] - cx) / fx, (start[1] - cy) / fy, 1.0)
    b = ((end[0] - cx) / fx, (end[1] - cy) / fy, 1.0)
    return normalize_vector(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0])


def mediapipe_to_optical(direction):
    """Both systems use +Z away from the camera; translation is irrelevant."""
    return normalize_vector(*direction)


def pointing_forward_normal(start, end, fx, fy, cx, cy):
    """Normal of the half-plane beyond the arm start in image direction."""
    a = normalize_vector((start[0]-cx)/fx, (start[1]-cy)/fy, 1.0)
    b = ((end[0]-cx)/fx, (end[1]-cy)/fy, 1.0)
    dot = sum(x*y for x, y in zip(a, b))
    return normalize_vector(*(b[k] - dot*a[k] for k in range(3)))


def robust_foreground_depth(values, min_support=3, band=0.025):
    """Nearest supported depth mode in metres; never interpolate two surfaces."""
    depths = sorted(float(d) for d in values
                    if math.isfinite(float(d)) and 0.15 < float(d) < 8.0)
    for index, depth in enumerate(depths):
        group = [d for d in depths[index:] if d - depth <= band]
        if len(group) >= min_support:
            return group[len(group) // 2]
    return None


def fit_projective_depth_line(samples, min_samples=5, max_error=0.035,
                              min_length=0.08, max_length=0.75):
    """RANSAC with perspective-correct interpolation along an image segment.

    Image position t is linear in x/z, y/z and 1/z, not in metric XYZ.
    Pair hypotheses and consensus reject isolated/background depth samples.
    The returned origin and end correspond to the observed segment endpoints.
    """
    points = [tuple(map(float, p)) for p in samples
              if len(p) == 4 and all(math.isfinite(float(v)) for v in p)
              and 0.15 < float(p[3]) < 8.0]
    if len(points) < min_samples:
        return None
    features = [(p[0], p[1]/p[3], p[2]/p[3], 1.0/p[3]) for p in points]

    def predict(intercept, slope, t):
        values = [intercept[k] + t*slope[k] for k in range(3)]
        if values[2] <= 0.0:
            return None
        return values[0]/values[2], values[1]/values[2], 1.0/values[2]

    best = []
    best_error = float('inf')
    for i, a in enumerate(features):
        for b in features[i+1:]:
            if abs(b[0] - a[0]) < 0.4:
                continue
            slope = [(b[k+1]-a[k+1])/(b[0]-a[0]) for k in range(3)]
            intercept = [a[k+1]-a[0]*slope[k] for k in range(3)]
            inliers, error = [], 0.0
            for index, p in enumerate(points):
                q = predict(intercept, slope, p[0])
                residual = math.dist(p[1:], q) if q else float('inf')
                if residual <= max_error:
                    inliers.append(index)
                    error += residual*residual
            if len(inliers) > len(best) or (len(inliers) == len(best) and error < best_error):
                best, best_error = inliers, error
    if len(best) < min_samples or len(best) < len(points)*0.65:
        return None
    selected = [features[i] for i in best]
    # Both ends must be observed. A short background patch is not an arm.
    if min(p[0] for p in selected) > 0.25 or max(p[0] for p in selected) < 0.75:
        return None
    mean_t = sum(p[0] for p in selected)/len(selected)
    denom = sum((p[0]-mean_t)**2 for p in selected)
    if denom < 1e-8:
        return None
    means = [sum(p[k+1] for p in selected)/len(selected) for k in range(3)]
    slope = [sum((p[0]-mean_t)*(p[k+1]-means[k]) for p in selected)/denom
             for k in range(3)]
    intercept = [means[k]-mean_t*slope[k] for k in range(3)]
    origin, end = predict(intercept, slope, 0.0), predict(intercept, slope, 1.0)
    if origin is None or end is None:
        return None
    length = math.dist(origin, end)
    if not min_length <= length <= max_length:
        return None
    rmse = math.sqrt(sum(math.dist(points[i][1:], predict(intercept, slope, points[i][0]))**2
                         for i in best)/len(best))
    if rmse > max_error:
        return None
    direction = tuple((end[k]-origin[k])/length for k in range(3))
    return origin, direction, rmse, len(best), length


def select_synchronized_pair(
    rgb_timestamps,
    depth_timestamps,
    max_delta_ns=80_000_000,
):
    """Select the newest RGB frame having an acceptably close depth frame."""
    best = None
    for rgb_index, rgb_stamp in enumerate(rgb_timestamps):
        for depth_index, depth_stamp in enumerate(depth_timestamps):
            delta = abs(int(rgb_stamp) - int(depth_stamp))
            if delta > max_delta_ns:
                continue
            key = (int(rgb_stamp), -delta)
            if best is None or key > best[0]:
                best = (key, rgb_index, depth_index, delta)
    if best is None:
        return None
    return best[1], best[2], best[3]


def is_extended_index_finger(
    wrist,
    index_mcp,
    index_pip,
    index_tip,
    min_segment_px=6.0,
):
    """Return whether a close-range hand has a usable extended index ray."""
    proximal = (
        index_pip[0] - index_mcp[0], index_pip[1] - index_mcp[1])
    distal = (
        index_tip[0] - index_pip[0], index_tip[1] - index_pip[1])
    proximal_length = math.hypot(*proximal)
    distal_length = math.hypot(*distal)
    if proximal_length < min_segment_px or distal_length < min_segment_px:
        return False
    straightness = (
        proximal[0] * distal[0] + proximal[1] * distal[1]) / (
            proximal_length * distal_length)
    wrist_to_pip = math.hypot(
        index_pip[0] - wrist[0], index_pip[1] - wrist[1])
    wrist_to_tip = math.hypot(
        index_tip[0] - wrist[0], index_tip[1] - wrist[1])
    return straightness >= 0.30 and wrist_to_tip >= wrist_to_pip + 2.0


def fit_parametric_line_3d(samples, min_samples=4, max_rmse=0.08):
    """Robustly fit point(t) = origin + t * slope to RGB-D arm samples.

    ``samples`` contains ``(t, x, y, z)`` entries ordered from elbow to the
    pointing endpoint.  Fitting all available points is substantially less
    sensitive to one background pixel than subtracting two depth samples.
    Returns ``(origin, unit_direction, rmse, inlier_count, length)`` or None.
    """
    valid = []
    for sample in samples:
        if len(sample) != 4 or not all(math.isfinite(float(v)) for v in sample):
            continue
        valid.append(tuple(float(v) for v in sample))
    if len(valid) < min_samples:
        return None

    def _fit(points):
        mean_t = sum(p[0] for p in points) / len(points)
        denom = sum((p[0] - mean_t) ** 2 for p in points)
        if denom < 1e-6:
            return None
        means = [sum(p[axis] for p in points) / len(points)
                 for axis in (1, 2, 3)]
        slope = [sum((p[0] - mean_t) * (p[axis] - means[axis - 1])
                     for p in points) / denom for axis in (1, 2, 3)]
        origin = [means[i] - slope[i] * mean_t for i in range(3)]
        residuals = []
        for point in points:
            predicted = [origin[i] + slope[i] * point[0] for i in range(3)]
            residuals.append(math.sqrt(sum(
                (point[i + 1] - predicted[i]) ** 2 for i in range(3))))
        return origin, slope, residuals

    initial = _fit(valid)
    if initial is None:
        return None
    sorted_residuals = sorted(initial[2])
    median_residual = sorted_residuals[len(sorted_residuals) // 2]
    deviations = sorted(abs(value - median_residual) for value in initial[2])
    mad = deviations[len(deviations) // 2]
    threshold = max(0.035, median_residual + 2.5 * max(mad, 0.005))
    inliers = [point for point, residual in zip(valid, initial[2])
               if residual <= threshold]
    if len(inliers) < min_samples:
        return None
    fitted = _fit(inliers)
    if fitted is None:
        return None
    origin, slope, residuals = fitted
    rmse = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    length = math.sqrt(sum(value * value for value in slope))
    t_span = max(p[0] for p in inliers) - min(p[0] for p in inliers)
    if rmse > max_rmse or length < 0.08 or length > 1.2 or t_span < 0.45:
        return None
    direction = tuple(value / length for value in slope)
    return tuple(origin), direction, rmse, len(inliers), length


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


def merge_multiscale_boxes(boxes, iou_threshold=0.7):
    """Class-aware NMS for [x1,y1,x2,y2,confidence,class] in camera pixels.

    Each image scale reports original-image coordinates. A scale is never an
    extra temporal vote. Different classes and disjoint instances are kept.
    """
    order = sorted(range(len(boxes)), key=lambda i: -boxes[i][4])
    keep = []
    for index in order:
        a = boxes[index]
        duplicate = False
        for previous in keep:
            b = boxes[previous]
            if a[5] != b[5]:
                continue
            area_a = max(0, a[2]-a[0])*max(0, a[3]-a[1])
            area_b = max(0, b[2]-b[0])*max(0, b[3]-b[1])
            intersection = max(0, min(a[2],b[2])-max(a[0],b[0])) * max(
                0, min(a[3],b[3])-max(a[1],b[1]))
            union = area_a+area_b-intersection
            if union > 0 and intersection/union > iou_threshold:
                duplicate = True
                break
        if not duplicate:
            keep.append(index)
    return keep

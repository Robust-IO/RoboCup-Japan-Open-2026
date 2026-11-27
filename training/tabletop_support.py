"""Observed tabletop support, not an exported furniture bounding box.

Unknown cells (including occlusions and holes) are unsafe. Only tiny sampling
gaps are filled, and that fill plus raster quantisation is subtracted from the
reported clearance. No convex hull or rectangle bridges unobserved space.
This is a conservative RGBD estimate, not an absolute calibration guarantee.
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


GRID_M = .005
EDGE_GAP_M = .03             # User-selected object OUTER gap, not centre inset.
EDGE_UNCERTAINTY_M = .01    # Additional position/settling allowance (r11).
# Filling one cell + sample position within its cell + EDT/query quantisation.
RASTER_ALLOWANCE_M = 3 * math.sqrt(2) * GRID_M
# Candidate is at a cell centre: 12mm alignment + 5mm radius variation + the
# maximum subcell offset at the later query (half a cell diagonal).
PLANNING_RESERVE_M = .017 + math.sqrt(2) * GRID_M / 2


def build_support(points, center, half, plane, destination_id, stamp_ns):
    points = np.asarray(points, float)
    center, half, plane = map(lambda x: np.asarray(x, float), (center, half, plane))
    if (points.ndim != 2 or points.shape[1] != 3 or center.shape != (2,)
            or half.shape != (2,) or plane.shape != (3,)
            or not np.isfinite([*center, *half, *plane]).all() or np.any(half <= 0)
            or type(stamp_ns) is not int or stamp_ns <= 0):
        raise ValueError('invalid_table_support_input')
    origin = np.floor((center - half) / GRID_M) * GRID_M - 2 * GRID_M
    shape_xy = np.ceil((center + half - origin) / GRID_M).astype(int) + 3
    if np.any(shape_xy > 512):
        raise ValueError('table_support_outside_reviewed_size')
    finite = np.isfinite(points).all(axis=1)
    sample = points[finite & (abs(points[:, :2] - center) < half).all(axis=1)]
    residual = sample[:, 2] - np.c_[sample[:, :2], np.ones(len(sample))] @ plane
    table = sample[abs(residual) <= .006]
    if len(table) < 150:
        raise ValueError('insufficient_observed_table_support')
    cells = np.floor((table[:, :2] - origin) / GRID_M).astype(int)
    raw = np.zeros(tuple(shape_xy[::-1]), bool)
    raw[cells[:, 1], cells[:, 0]] = True
    # Fill at most one 5mm cell in each direction, NOT arbitrary depth holes.
    mask = np.zeros_like(raw)
    padded = np.pad(raw, 1)
    for dy in range(3):
        for dx in range(3):
            mask |= padded[dy:dy + raw.shape[0], dx:dx + raw.shape[1]]
    return dict(version=1, destination_id=destination_id, source_stamp_ns=stamp_ns,
                resolution_m=GRID_M, origin_xy_m=origin.tolist(),
                mask_rows=[''.join(row.astype(np.uint8).astype(str)) for row in mask],
                raster_allowance_m=RASTER_ALLOWANCE_M,
                basis='observed_table_plane_unknown_is_unsafe')


class TableSupport:
    def __init__(self, model, destination_id):
        if (not isinstance(model, dict) or model.get('version') != 1
                or model.get('destination_id') != destination_id
                or model.get('resolution_m') != GRID_M
                or model.get('raster_allowance_m') != RASTER_ALLOWANCE_M
                or model.get('basis') != 'observed_table_plane_unknown_is_unsafe'
                or type(model.get('source_stamp_ns')) is not int or model['source_stamp_ns'] <= 0):
            raise ValueError('measured_table_support_required')
        self.origin = np.asarray(model.get('origin_xy_m'), float)
        rows = model.get('mask_rows')
        if (self.origin.shape != (2,) or not np.isfinite(self.origin).all()
                or not isinstance(rows, list) or not 3 <= len(rows) <= 512
                or not isinstance(rows[0], str) or not 3 <= len(rows[0]) <= 512
                or any(not isinstance(row, str) or len(row) != len(rows[0])
                       or set(row) - {'0', '1'} for row in rows)):
            raise ValueError('invalid_table_support_mask')
        self.mask = np.array([list(row) for row in rows]) == '1'
        # Explicit zero padding prevents an all-ones raster having no boundary.
        distance = distance_transform_edt(np.pad(self.mask, 1))[1:-1, 1:-1]
        self.distance_m = np.maximum(0., distance * GRID_M - RASTER_ALLOWANCE_M)

    def query(self, xy, radius):
        xy = np.asarray(xy, float)
        if xy.shape != (2,) or not np.isfinite([*xy, radius]).all() or not .015 <= radius <= .09:
            raise ValueError('invalid_object_edge_query')
        cell = np.floor((xy - self.origin) / GRID_M).astype(int)
        inside = (cell >= 0).all() and cell[0] < self.mask.shape[1] and cell[1] < self.mask.shape[0]
        distance = float(self.distance_m[cell[1], cell[0]]) if inside else 0.
        clearance = distance - radius
        return dict(safe=bool(clearance >= EDGE_GAP_M + EDGE_UNCERTAINTY_M),
                    center_to_unsupported_m=distance, object_radius_m=float(radius),
                    outer_clearance_lower_estimate_m=float(clearance),
                    minimum_outer_gap_m=EDGE_GAP_M, uncertainty_allowance_m=EDGE_UNCERTAINTY_M,
                    basis='observed_table_support_not_furniture_aabb')

    def safe_centers(self, radius, extra_clearance_m=0.):
        if (not math.isfinite(radius) or not .015 <= radius <= .09
                or not math.isfinite(extra_clearance_m) or not 0 <= extra_clearance_m <= .03):
            raise ValueError('invalid_object_edge_query')
        v, u = np.nonzero(self.distance_m >= radius + EDGE_GAP_M + EDGE_UNCERTAINTY_M + extra_clearance_m)
        return self.origin + np.c_[u + .5, v + .5] * GRID_M


def support_from_plan(plan):
    policy = plan.get('table_edge_policy')
    if policy != dict(version=1, minimum_outer_gap_m=EDGE_GAP_M,
                      uncertainty_allowance_m=EDGE_UNCERTAINTY_M,
                      selection='shortest_reachable_approach_after_clearance_passes'):
        raise ValueError('measured_table_edge_policy_required_replan_legacy_placement')
    support = TableSupport(plan.get('observed_table_support'), plan['destination_id'])
    if plan.get('source_depth_stamp_ns') != plan['observed_table_support']['source_stamp_ns']:
        raise ValueError('table_support_plan_capture_mismatch')
    return support

"""Generate competition environment configuration from Unity exports."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import shutil
import sys
from typing import Any, Dict, List, Sequence, Tuple

from .occupancy_map import OccupancyMap, point_in_polygon
from .semantic_map import load, require_valid


Pose = Tuple[float, float, float]
Point = Tuple[float, float]
DEFAULT_LAYOUTS = ('LayoutA', 'LayoutB', 'LayoutC', 'LayoutD')


def _number(value: float) -> str:
    value = 0.0 if abs(value) < 0.0000005 else value
    return f'{value:.6f}'.rstrip('0').rstrip('.')


def _pose_yaml(pose: Pose, room: str = '') -> str:
    fields = []
    if room:
        fields.append(f'room: {room}')
    fields.extend((
        f'x: {_number(pose[0])}',
        f'y: {_number(pose[1])}',
        f'yaw: {_number(pose[2])}',
    ))
    return '{' + ', '.join(fields) + '}'


def _polygon(room: Dict[str, Any]) -> List[Point]:
    return [(float(item['x']), float(item['y'])) for item in room['polygon']]


def _center(points: Sequence[Point]) -> Point:
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def _inset_polygon(points: Sequence[Point], distance: float = 0.05) -> List[Point]:
    """Move vertices slightly toward the center to remove trigger overlap."""
    center = _center(points)
    result = []
    for point in points:
        length = _distance(point, center)
        if length <= distance:
            raise ValueError('room polygon is too small to inset safely')
        amount = distance / length
        result.append((
            point[0] + (center[0] - point[0]) * amount,
            point[1] + (center[1] - point[1]) * amount,
        ))
    return result


def _distance(left: Point, right: Point) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


def _boundary_distance(point: Point, polygon: Sequence[Point]) -> float:
    best = math.inf
    previous = len(polygon) - 1
    for current, end in enumerate(polygon):
        start = polygon[previous]
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        length_squared = delta_x * delta_x + delta_y * delta_y
        if length_squared == 0.0:
            nearest = start
        else:
            amount = max(0.0, min(1.0, (
                (point[0] - start[0]) * delta_x
                + (point[1] - start[1]) * delta_y
            ) / length_squared))
            nearest = (
                start[0] + amount * delta_x,
                start[1] + amount * delta_y,
            )
        best = min(best, _distance(point, nearest))
        previous = current
    return best


def _search_poses(
    grid: OccupancyMap,
    polygon: Sequence[Point],
    count: int = 4,
) -> List[Pose]:
    candidates = [
        item for item in grid.safe_cells_in_polygon(polygon, 0.35)
        if _boundary_distance(grid.index_to_world(item), polygon) >= 0.25
    ]
    if not candidates:
        candidates = [
            item
            for item in grid.safe_cells_in_polygon(polygon, 0.20, 0.10)
            if _boundary_distance(grid.index_to_world(item), polygon) >= 0.10
        ]
    if not candidates:
        raise ValueError('room contains no collision-safe free search point')
    room_center = _center(polygon)
    positions = {index: grid.index_to_world(index) for index in candidates}
    first = max(
        candidates,
        key=lambda item: (
            grid.clearance(item) - 0.05 * _distance(positions[item], room_center)
        ),
    )
    selected = [first]
    while len(selected) < count:
        remaining = [item for item in candidates if item not in selected]
        if not remaining:
            break
        next_item = max(
            remaining,
            key=lambda item: (
                min(
                    _distance(positions[item], positions[chosen])
                    for chosen in selected
                ),
                grid.clearance(item),
            ),
        )
        separation = min(
            _distance(positions[next_item], positions[chosen])
            for chosen in selected
        )
        if separation < 0.80:
            break
        selected.append(next_item)
    result = []
    for index in selected:
        x, y = positions[index]
        yaw = math.atan2(room_center[1] - y, room_center[0] - x)
        result.append((x, y, yaw))
    return result


def _approach_poses(
    grid: OccupancyMap,
    polygon: Sequence[Point],
    destination: Dict[str, Any],
    count: int = 3,
) -> List[Pose]:
    bounds = (
        destination.get('oriented_bounds')
        or destination.get('axis_aligned_bounds')
        or {}
    )
    bounds_center = bounds.get('center') or {}
    model_pose = destination.get('model_pose') or destination['pose']
    target = (
        float(bounds_center.get('x', model_pose['x'])),
        float(bounds_center.get('y', model_pose['y'])),
    )
    size = bounds.get('size') or {}
    object_radius = 0.5 * max(
        abs(float(size.get('x', 0.0))),
        abs(float(size.get('y', 0.0))),
    )
    preferred = max(0.65, object_radius + 0.50)
    minimum = max(0.45, object_radius + 0.30)
    maximum = preferred + 0.65
    candidates = [
        item
        for item in grid.safe_cells_in_polygon(polygon, 0.35, 0.10)
        if _boundary_distance(grid.index_to_world(item), polygon) >= 0.10
    ]
    candidates = [
        index for index in candidates
        if minimum <= _distance(grid.index_to_world(index), target) <= maximum
    ]
    if not candidates:
        candidates = [
            item
            for item in grid.safe_cells_in_polygon(polygon, 0.20, 0.10)
            if _boundary_distance(grid.index_to_world(item), polygon) >= 0.05
        ]
        candidates = [
            index for index in candidates
            if 0.35 <= _distance(grid.index_to_world(index), target)
            <= maximum + 0.50
        ]
    if not candidates:
        raise ValueError(
            f"destination {destination['id']!r} has no safe approach pose"
        )
    has_model_front = (
        destination.get('front_source') == 'model_transform_forward'
        and max(
            abs(float(size.get('x', 0.0))),
            abs(float(size.get('y', 0.0))),
        ) >= 0.10
    )
    if not has_model_front:
        # Preserve schema-v1 behavior for already exported competition maps.
        candidates.sort(key=lambda item: (
            abs(_distance(grid.index_to_world(item), target) - preferred),
            -grid.clearance(item),
        ))
        selected: List[int] = []
        selected_angles: List[float] = []
        for index in candidates:
            point = grid.index_to_world(index)
            angle = math.atan2(point[1] - target[1], point[0] - target[0])
            if all(
                abs(math.atan2(
                    math.sin(angle - old), math.cos(angle - old)
                )) >= math.radians(55.0)
                for old in selected_angles
            ):
                selected.append(index)
                selected_angles.append(angle)
                if len(selected) == count:
                    break
        if len(selected) == 1:
            first_point = grid.index_to_world(selected[0])
            alternatives = [
                item for item in candidates
                if _distance(grid.index_to_world(item), first_point) >= 0.25
            ]
            if alternatives:
                selected.append(max(
                    alternatives,
                    key=lambda item: _distance(
                        grid.index_to_world(item), first_point
                    ),
                ))
        result = []
        for index in selected:
            x, y = grid.index_to_world(index)
            result.append((
                x, y, math.atan2(target[1] - y, target[0] - x)
            ))
        return result

    front_yaw = float(destination['front_yaw'])

    def angle_difference(left: float, right: float) -> float:
        return abs(math.atan2(
            math.sin(left - right), math.cos(left - right)
        ))

    def best_in_direction(direction: float):
        directional = []
        for index in candidates:
            point = grid.index_to_world(index)
            angle = math.atan2(point[1] - target[1], point[0] - target[0])
            error = angle_difference(angle, direction)
            if error <= math.radians(52.0):
                directional.append((index, point, error))
        if not directional:
            return None
        return min(directional, key=lambda item: (
            item[2],
            abs(_distance(item[1], target) - preferred),
            -_boundary_distance(item[1], polygon),
            -grid.clearance(item[0]),
        ))[0]

    # Transform.forward is exported as the furniture front. Runtime retry
    # order is deliberately front, safer side, other side, and only then a
    # geometry-only fallback. This keeps wall-side trash-bin poses last.
    front = best_in_direction(front_yaw)
    side_options = [
        best_in_direction(front_yaw + math.pi / 2.0),
        best_in_direction(front_yaw - math.pi / 2.0),
    ]
    side_options = [item for item in side_options if item is not None]
    side_options.sort(key=lambda item: (
        _boundary_distance(grid.index_to_world(item), polygon),
        grid.clearance(item),
    ), reverse=True)
    rear = best_in_direction(front_yaw + math.pi)
    ordered = ([front] if front is not None else []) + side_options
    if rear is not None:
        ordered.append(rear)

    selected: List[int] = []
    for index in ordered:
        if index not in selected and all(
            _distance(grid.index_to_world(index), grid.index_to_world(old))
            >= 0.25
            for old in selected
        ):
            selected.append(index)
        if len(selected) == count:
            break
    if len(selected) < count:
        remaining = sorted(candidates, key=lambda item: (
            -_boundary_distance(grid.index_to_world(item), polygon),
            -grid.clearance(item),
            abs(_distance(grid.index_to_world(item), target) - preferred),
        ))
        for index in remaining:
            if all(
                _distance(grid.index_to_world(index), grid.index_to_world(old))
                >= 0.25
                for old in selected
            ):
                selected.append(index)
            if len(selected) == count:
                break
    result = []
    for index in selected:
        x, y = grid.index_to_world(index)
        result.append((x, y, math.atan2(target[1] - y, target[0] - x)))
    return result


def _safe_initial_pose(
    grid: OccupancyMap,
    raw_pose: Dict[str, Any],
    rooms: Sequence[Dict[str, Any]],
) -> Tuple[Pose, bool]:
    original = (
        float(raw_pose['x']),
        float(raw_pose['y']),
        float(raw_pose['yaw']),
    )
    original_index = grid.world_to_index(original[0], original[1])
    if grid.clearance(original_index) >= 0.25:
        return original, False
    candidates = []
    for room in rooms:
        candidates.extend(grid.safe_cells_in_polygon(_polygon(room), 0.25, 0.10))
    if not candidates:
        raise ValueError('no safe replacement for robot_initial_pose')
    nearest = min(
        candidates,
        key=lambda item: _distance(
            grid.index_to_world(item), (original[0], original[1])
        ),
    )
    x, y = grid.index_to_world(nearest)
    return (x, y, original[2]), True


def _line_is_safe(
    grid: OccupancyMap,
    start: Point,
    end: Point,
    minimum_clearance: float,
) -> bool:
    length = _distance(start, end)
    steps = max(1, math.ceil(length / (0.5 * grid.resolution)))
    for step in range(steps + 1):
        amount = step / steps
        index = grid.world_to_index(
            start[0] + amount * (end[0] - start[0]),
            start[1] + amount * (end[1] - start[1]),
        )
        if grid.clearance(index) < minimum_clearance:
            return False
    return True


def _door_pose(doorway: Dict[str, Any], field: str) -> Point:
    pose = doorway[field]
    return float(pose['x']), float(pose['y'])


def _door_center(doorway: Dict[str, Any]) -> Point:
    return _door_pose(doorway, 'center')


def _path_length(grid: OccupancyMap, path: Sequence[int]) -> float:
    if len(path) < 2:
        return 0.0
    points = [grid.index_to_world(index) for index in path]
    return sum(
        _distance(points[index - 1], points[index])
        for index in range(1, len(points))
    )


def _door_centered_waypoints(
    grid: OccupancyMap,
    start: Pose,
    goal: Pose,
    doorways: Sequence[Dict[str, Any]],
    from_room: str,
    to_room: str,
) -> List[Pose]:
    source_doors = [
        item for item in doorways if item.get('room') == from_room
    ]
    target_doors = [
        item for item in doorways if item.get('room') == to_room
    ]
    if not source_doors or not target_doors:
        return []

    def poses_from_points(points: Sequence[Point]) -> List[Pose]:
        deduplicated = []
        for point in points:
            if not deduplicated or _distance(
                point, deduplicated[-1]
            ) >= 0.10:
                deduplicated.append(point)
        result = []
        for index, point in enumerate(deduplicated):
            following = (
                deduplicated[index + 1]
                if index + 1 < len(deduplicated)
                else goal[:2]
            )
            result.append((
                point[0], point[1],
                math.atan2(
                    following[1] - point[1],
                    following[0] - point[0],
                ),
            ))
        return result

    # Adjacent rooms export the same physical opening from both sides. Use
    # exactly one representation; combining both would cross the doorway and
    # then immediately drive backward through it.
    direct_candidates = []
    for doorway in source_doors:
        if doorway.get('connected_room') != to_room:
            continue
        if not any(
            _distance(_door_center(doorway), _door_center(target)) <= 0.60
            for target in target_doors
        ):
            continue
        inside = _door_pose(doorway, 'inside_pose')
        outside = _door_pose(doorway, 'outside_pose')
        start_path = grid.shortest_safe_path(start[:2], inside, 0.20)
        goal_path = grid.shortest_safe_path(outside, goal[:2], 0.20)
        if start_path and goal_path and _line_is_safe(
            grid, inside, outside, 0.14
        ):
            direct_candidates.append((
                _path_length(grid, start_path)
                + _distance(inside, outside)
                + _path_length(grid, goal_path),
                [inside, outside],
            ))
    for doorway in target_doors:
        if doorway.get('connected_room') != from_room:
            continue
        outside = _door_pose(doorway, 'outside_pose')
        inside = _door_pose(doorway, 'inside_pose')
        start_path = grid.shortest_safe_path(start[:2], outside, 0.20)
        goal_path = grid.shortest_safe_path(inside, goal[:2], 0.20)
        if start_path and goal_path and _line_is_safe(
            grid, outside, inside, 0.14
        ):
            direct_candidates.append((
                _path_length(grid, start_path)
                + _distance(outside, inside)
                + _path_length(grid, goal_path),
                [outside, inside],
            ))
    if direct_candidates:
        return poses_from_points(min(
            direct_candidates, key=lambda item: item[0]
        )[1])

    best = None
    for source in source_doors:
        source_inside = _door_pose(source, 'inside_pose')
        source_outside = _door_pose(source, 'outside_pose')
        for target in target_doors:
            if (
                source.get('connected_room') == to_room
                or target.get('connected_room') == from_room
                or _distance(
                    _door_center(source), _door_center(target)
                ) <= 0.60
            ):
                continue
            target_outside = _door_pose(target, 'outside_pose')
            target_inside = _door_pose(target, 'inside_pose')
            if not _line_is_safe(
                grid, source_inside, source_outside, 0.14
            ) or not _line_is_safe(
                grid, target_outside, target_inside, 0.14
            ):
                continue
            start_path = grid.shortest_safe_path(
                start[:2], source_inside, 0.20
            )
            corridor_path = grid.shortest_safe_path(
                source_outside, target_outside, 0.20
            )
            goal_path = grid.shortest_safe_path(
                target_inside, goal[:2], 0.20
            )
            if not start_path or not corridor_path or not goal_path:
                continue
            cost = (
                _path_length(grid, start_path)
                + _distance(source_inside, source_outside)
                + _path_length(grid, corridor_path)
                + _distance(target_outside, target_inside)
                + _path_length(grid, goal_path)
            )
            candidate = (
                cost, source_inside, source_outside,
                corridor_path, target_outside, target_inside,
            )
            if best is None or candidate[0] < best[0]:
                best = candidate
    if best is None:
        return []

    _, source_inside, source_outside, corridor_path, \
        target_outside, target_inside = best
    corridor_points = [
        grid.index_to_world(index) for index in corridor_path
    ]
    sparse_corridor: List[Point] = []
    anchor = 0
    while anchor + 1 < len(corridor_points):
        furthest = anchor + 1
        for candidate in range(anchor + 2, len(corridor_points)):
            if _line_is_safe(
                grid,
                corridor_points[anchor],
                corridor_points[candidate],
                0.20,
            ):
                furthest = candidate
            else:
                break
        if furthest < len(corridor_points) - 1:
            sparse_corridor.append(corridor_points[furthest])
        anchor = furthest

    points = [
        source_inside,
        source_outside,
        *sparse_corridor,
        target_outside,
        target_inside,
    ]
    return poses_from_points(points)


def _route_waypoint_candidates(
    grid: OccupancyMap,
    start: Pose,
    goal: Pose,
    target_polygon: Sequence[Point],
    doorways: Sequence[Dict[str, Any]],
    from_room: str,
    to_room: str,
    maximum_routes: int = 3,
) -> List[List[Pose]]:
    target_doors = [
        item for item in doorways if item.get('room') == to_room
    ]
    candidates = []
    for target in target_doors:
        filtered = [
            item for item in doorways
            if item.get('room') != to_room or item is target
        ]
        route = _door_centered_waypoints(
            grid, start, goal, filtered, from_room, to_room
        )
        if not route:
            continue
        signature = tuple(
            (round(pose[0], 2), round(pose[1], 2)) for pose in route
        )
        if any(existing[0] == signature for existing in candidates):
            continue
        length = _distance(start[:2], route[0][:2])
        length += sum(
            _distance(route[index - 1][:2], route[index][:2])
            for index in range(1, len(route))
        )
        length += _distance(route[-1][:2], goal[:2])
        candidates.append((signature, length, route))
    if candidates:
        candidates.sort(key=lambda item: item[1])
        return [item[2] for item in candidates[:maximum_routes]]
    # A target doorway can be unusable while the source doorway is valid.
    # Preserve the source's straight exit before simplifying the rest of the
    # path; otherwise simplification can put a turn inside the door frame.
    exits = []
    for source in doorways:
        if source.get('room') != from_room:
            continue
        inside = _door_pose(source, 'inside_pose')
        outside = _door_pose(source, 'outside_pose')
        approach = grid.shortest_safe_path(start[:2], inside, 0.20)
        onward = grid.shortest_safe_path(outside, goal[:2], 0.30)
        if not approach or not onward or not _line_is_safe(
            grid, inside, outside, 0.30
        ):
            continue
        yaw = math.atan2(outside[1] - inside[1], outside[0] - inside[0])
        tail = _route_waypoints(
            grid, (*outside, yaw), goal, target_polygon
        )
        route = [(*inside, yaw), (*outside, yaw), *tail]
        exits.append((
            _path_length(grid, approach) + _distance(inside, outside)
            + _path_length(grid, onward), route,
        ))
    if exits:
        exits.sort(key=lambda item: item[0])
        return [item[1] for item in exits[:maximum_routes]]
    fallback = _route_waypoints(
        grid, start, goal, target_polygon
    )
    return [fallback] if fallback else []


def _route_waypoints(
    grid: OccupancyMap,
    start: Pose,
    goal: Pose,
    target_polygon: Sequence[Point],
    minimum_clearance: float = 0.30,
    doorways: Sequence[Dict[str, Any]] = (),
    from_room: str = '',
    to_room: str = '',
) -> List[Pose]:
    """Find sparse, collision-safe turning points for a cross-room route."""
    if doorways and from_room and to_room:
        centered = _door_centered_waypoints(
            grid, start, goal, doorways, from_room, to_room
        )
        if centered:
            return centered
    path = grid.shortest_safe_path(start[:2], goal[:2], minimum_clearance)
    if not path:
        return []
    points = [grid.index_to_world(index) for index in path]
    sparse_indices = [0]
    anchor = 0
    while anchor + 1 < len(points):
        furthest = anchor + 1
        for candidate in range(anchor + 2, len(points)):
            if _line_is_safe(
                grid, points[anchor], points[candidate], minimum_clearance
            ):
                furthest = candidate
            else:
                break
        sparse_indices.append(furthest)
        anchor = furthest

    crossing = next((
        index for index in range(1, len(points))
        if point_in_polygon(points[index], target_polygon)
        and not point_in_polygon(points[index - 1], target_polygon)
    ), None)
    if crossing is not None:
        def offset_index(origin: int, direction: int, distance: float) -> int:
            travelled = 0.0
            current = origin
            while 0 <= current + direction < len(points) and travelled < distance:
                following = current + direction
                travelled += _distance(points[current], points[following])
                current = following
            return current

        outside = offset_index(crossing, -1, 0.70)
        inside = offset_index(crossing, 1, 0.70)
        # Replace incidental simplification corners around the doorway with a
        # deliberate approach and exit pair, leaving room for goal tolerance.
        sparse_indices = [
            index for index in sparse_indices
            if abs(index - crossing) * grid.resolution > 0.90
        ]
        sparse_indices.extend((outside, inside, len(points) - 1))

    sparse_indices = sorted(set(sparse_indices))
    # The source and room search pose are not intermediate waypoints.
    result = []
    for index in range(1, len(sparse_indices) - 1):
        current = points[sparse_indices[index]]
        following = points[sparse_indices[index + 1]]
        result.append((
            current[0],
            current[1],
            math.atan2(following[1] - current[1], following[0] - current[0]),
        ))
    return result


def _render_environment(
    layout: str,
    document: Dict[str, Any],
    grid: OccupancyMap,
) -> Tuple[str, Dict[str, Any]]:
    rooms = {room['id']: room for room in document['rooms']}
    initial_pose, initial_snapped = _safe_initial_pose(
        grid, document['robot_initial_pose'], document['rooms']
    )
    lines = [
        f'internal_name: {document["environment"]}',
        'map: package://handyman_rebuild_ros2/maps/'
        f'{layout}/map.yaml',
        f'initial_pose: {_pose_yaml(initial_pose)}',
        'rooms:',
    ]
    room_components = {}
    search_poses_by_room = {}
    for room_name in sorted(rooms):
        polygon = _polygon(rooms[room_name])
        config_polygon = _inset_polygon(polygon)
        points = ', '.join(
            f'[{_number(point[0])}, {_number(point[1])}]'
            for point in config_polygon
        )
        search_poses = _search_poses(grid, polygon)
        search_poses_by_room[room_name] = search_poses
        lines.extend((
            f'  {room_name}:',
            f'    region: [{points}]',
            '    search_points:',
        ))
        lines.extend(f'      - {_pose_yaml(pose)}' for pose in search_poses)
        room_components[room_name] = grid.component(
            search_poses[0][0], search_poses[0][1]
        )

    lines.append('routes:')
    route_counts = {}
    for from_room in sorted(rooms):
        for to_room in sorted(rooms):
            if from_room == to_room:
                continue
            route_candidates = _route_waypoint_candidates(
                grid,
                search_poses_by_room[from_room][0],
                search_poses_by_room[to_room][0],
                _polygon(rooms[to_room]),
                document.get('doorways') or (),
                from_room,
                to_room,
            )
            route_counts[f'{from_room}->{to_room}'] = len(
                route_candidates
            )
            if not route_candidates:
                continue
            for waypoints in route_candidates:
                lines.extend((
                    f'  - from: {from_room}',
                    f'    to: {to_room}',
                    '    waypoints:',
                ))
                lines.extend(
                    f'      - {_pose_yaml(pose)}' for pose in waypoints
                )

    grouped = defaultdict(list)
    approach_counts = {}
    for destination in document['destinations']:
        room_name = destination['room']
        poses = _approach_poses(
            grid, _polygon(rooms[room_name]), destination
        )
        semantic_class = destination['semantic_class']
        grouped[semantic_class].extend((room_name, pose) for pose in poses)
        approach_counts[destination['id']] = len(poses)
    lines.append('destinations:')
    for name in sorted(grouped):
        lines.append(f'  {name}:')
        for room_name, pose in grouped[name]:
            lines.append(f'    - {_pose_yaml(pose, room_name)}')

    robot_component = grid.component(initial_pose[0], initial_pose[1])
    disconnected = sorted(
        room for room, component in room_components.items()
        if component >= 0 and component != robot_component
    )
    report = {
        'layout': layout,
        'environment': document['environment'],
        'grid': {
            'width': grid.width,
            'height': grid.height,
            'resolution': grid.resolution,
            'free_components': sorted(grid.component_sizes, reverse=True),
        },
        'initial_pose_snapped': initial_snapped,
        'room_region_inset_m': 0.05,
        'robot_component': robot_component,
        'room_components': room_components,
        'rooms_disconnected_from_robot': disconnected,
        'search_point_counts': {
            name: len(search_poses_by_room[name]) for name in rooms
        },
        'route_waypoint_counts': route_counts,
        'doorway_counts': {
            room: sum(
                1 for item in document.get('doorways', [])
                if item.get('room') == room
            )
            for room in rooms
        },
        'destination_approach_counts': approach_counts,
    }
    return '\n'.join(lines) + '\n', report


def import_layouts(
    export_root: Path,
    package_root: Path,
    layouts: Sequence[str] = DEFAULT_LAYOUTS,
) -> Dict[str, Any]:
    export_root = Path(export_root).expanduser().resolve()
    package_root = Path(package_root).expanduser().resolve()
    config_root = package_root / 'config' / 'environments'
    maps_root = package_root / 'maps'
    if not (package_root / 'package.xml').is_file():
        raise ValueError(f'not a ROS package directory: {package_root}')
    config_root.mkdir(parents=True, exist_ok=True)
    maps_root.mkdir(parents=True, exist_ok=True)
    reports = []
    for layout in layouts:
        source = export_root / layout
        required = ('map.pgm', 'map.yaml', 'semantic_map.json')
        missing = [name for name in required if not (source / name).is_file()]
        if missing:
            raise ValueError(f'{layout} is missing: {", ".join(missing)}')
        document = load(str(source / 'semantic_map.json'))
        require_valid(document)
        if document['environment'] != layout:
            raise ValueError(
                f'{layout} contains environment {document["environment"]!r}'
            )
        grid = OccupancyMap(source / 'map.yaml')
        yaml_text, report = _render_environment(layout, document, grid)
        target = maps_root / layout
        target.mkdir(parents=True, exist_ok=True)
        for name in ('map.pgm', 'map.yaml', 'semantic_map.json', 'preview.png'):
            source_file = source / name
            if source_file.is_file():
                shutil.copy2(source_file, target / name)
        config_name = f'{layout[:6].lower()}_{layout[6:].lower()}.yaml'
        (config_root / config_name).write_text(yaml_text, encoding='utf-8')
        reports.append(report)
    aggregate = {
        'schema_version': 1,
        'source': str(export_root),
        'layouts': reports,
    }
    (maps_root / 'import_report.json').write_text(
        json.dumps(aggregate, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    return aggregate


def main(argv: Sequence[str] = None) -> int:
    parser = argparse.ArgumentParser(
        description='Import Unity Handyman maps into handyman_rebuild_ros2.'
    )
    parser.add_argument('export_root', help='directory containing LayoutA-D')
    parser.add_argument('package_root', help='handyman_rebuild_ros2 source dir')
    parser.add_argument(
        '--layouts', nargs='+', default=list(DEFAULT_LAYOUTS),
        help='layout directories to import (default: LayoutA LayoutB LayoutC LayoutD)',
    )
    args = parser.parse_args(argv)
    try:
        report = import_layouts(
            Path(args.export_root), Path(args.package_root), args.layouts
        )
    except (OSError, ValueError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1
    for item in report['layouts']:
        disconnected = item['rooms_disconnected_from_robot']
        suffix = (
            ' disconnected rooms: ' + ', '.join(disconnected)
            if disconnected else ' all room centers connected'
        )
        print(
            f"Imported {item['layout']}: "
            f"{item['grid']['width']}x{item['grid']['height']};{suffix}"
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

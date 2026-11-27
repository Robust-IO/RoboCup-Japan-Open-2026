import json

from handyman_map_tools.import_unity_maps import (
    import_layouts,
    _approach_poses,
    _route_waypoints,
    _route_waypoint_candidates,
)
from handyman_map_tools.occupancy_map import OccupancyMap


def _write_export(root):
    layout = root / 'LayoutA'
    layout.mkdir(parents=True)
    width = height = 120
    pixels = bytearray([254] * (width * height))
    for row in range(height):
        for column in range(width):
            if row in (0, height - 1) or column in (0, width - 1):
                pixels[row * width + column] = 0
    (layout / 'map.pgm').write_bytes(
        f'P5\n{width} {height}\n255\n'.encode('ascii') + pixels
    )
    (layout / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
        encoding='utf-8',
    )
    room_bounds = {
        'living_room': (1.0, 1.0, 5.0, 5.0),
        'kitchen': (6.0, 1.0, 10.0, 5.0),
        'bedroom': (1.0, 6.0, 5.0, 10.0),
        'lobby': (6.0, 6.0, 10.0, 10.0),
    }
    rooms = []
    for name, (x0, y0, x1, y1) in room_bounds.items():
        rooms.append({
            'id': name,
            'polygon': [
                {'x': x0, 'y': y0}, {'x': x1, 'y': y0},
                {'x': x1, 'y': y1}, {'x': x0, 'y': y1},
            ],
        })
    document = {
        'schema_version': 1,
        'environment': 'LayoutA',
        'coordinate_system': 'ros_map',
        'rooms': rooms,
        'destinations': [{
            'id': 'table#living_room',
            'semantic_class': 'table',
            'room': 'living_room',
            'pose': {'x': 3.0, 'y': 3.0, 'yaw': 0.0},
            'axis_aligned_bounds': {
                'size': {'x': 0.5, 'y': 0.5},
            },
        }],
        'object_spawn_candidates': [{
            'id': 'candidate01',
            'room': 'living_room',
            'pose': {'x': 2.0, 'y': 2.0, 'yaw': 0.0},
        }],
        'robot_initial_pose': {'x': 2.0, 'y': 2.0, 'yaw': 0.0},
        'messages': [],
    }
    (layout / 'semantic_map.json').write_text(
        json.dumps(document), encoding='utf-8'
    )


def test_occupancy_map_coordinate_round_trip(tmp_path):
    _write_export(tmp_path)
    grid = OccupancyMap(tmp_path / 'LayoutA' / 'map.yaml')
    index = grid.world_to_index(2.05, 3.05)
    assert grid.is_free(index)
    x, y = grid.index_to_world(index)
    assert abs(x - 2.05) < 1e-6
    assert abs(y - 3.05) < 1e-6


def test_cross_room_route_has_separated_door_approach_and_exit(tmp_path):
    _write_export(tmp_path)
    grid = OccupancyMap(tmp_path / 'LayoutA' / 'map.yaml')
    target_room = [(6.0, 1.0), (10.0, 1.0), (10.0, 5.0), (6.0, 5.0)]
    waypoints = _route_waypoints(
        grid, (2.0, 2.0, 0.0), (8.0, 2.0, 0.0), target_room
    )
    assert len(waypoints) >= 2
    outside = next(point for point in waypoints if point[0] < 6.0)
    inside = next(point for point in waypoints if point[0] > 6.0)
    assert inside[0] - outside[0] >= 1.0


def test_fallback_preserves_source_exit_when_target_door_missing(tmp_path):
    _write_export(tmp_path)
    grid = OccupancyMap(tmp_path / 'LayoutA' / 'map.yaml')
    door = {
        'room': 'bedroom',
        'inside_pose': {'x': 4.0, 'y': 7.0},
        'outside_pose': {'x': 4.0, 'y': 5.6},
    }
    routes = _route_waypoint_candidates(
        grid, (3.0, 8.0, 0.0), (2.0, 2.0, 0.0),
        [(1., 1.), (5., 1.), (5., 5.), (1., 5.)],
        [door], 'bedroom', 'living_room',
    )
    assert routes
    assert routes[0][0][:2] == (4.0, 7.0)
    assert routes[0][1][:2] == (4.0, 5.6)
    assert abs(routes[0][0][2] + 1.57079632679) < 1e-6
    assert routes[0][1][2] == routes[0][0][2]


def test_destination_approach_prefers_model_front_then_open_side(tmp_path):
    _write_export(tmp_path)
    grid = OccupancyMap(tmp_path / 'LayoutA' / 'map.yaml')
    polygon = [(1.0, 1.0), (5.0, 1.0), (5.0, 5.0), (1.0, 5.0)]
    destination = {
        'id': 'trash_box_for_burnable#living_room',
        'pose': {'x': 3.0, 'y': 1.6, 'yaw': 0.0},
        'model_pose': {'x': 3.0, 'y': 1.6, 'yaw': 0.0},
        'front_yaw': 0.0,
        'front_source': 'model_transform_forward',
        'oriented_bounds': {
            'center': {'x': 3.0, 'y': 1.6},
            'size': {'x': 0.5, 'y': 0.5},
        },
    }

    poses = _approach_poses(grid, polygon, destination)

    assert poses[0][0] > 3.0  # Unity model forward / furniture front
    assert poses[1][1] > 1.6  # open side precedes wall-side candidate
    assert poses[2][1] < 1.6


def test_multiple_doorways_choose_shorter_centered_route(tmp_path):
    _write_export(tmp_path)
    grid = OccupancyMap(tmp_path / 'LayoutA' / 'map.yaml')

    def door(room, connected, number, inside_x, outside_x, y, yaw):
        return {
            'id': f'{room}_entrance_{number}',
            'room': room,
            'connected_room': connected,
            'width': 0.9,
            'center': {'x': 5.5, 'y': y, 'yaw': yaw},
            'inside_pose': {'x': inside_x, 'y': y, 'yaw': yaw},
            'outside_pose': {'x': outside_x, 'y': y, 'yaw': yaw},
        }

    doorways = [
        door('living_room', 'kitchen', 1, 4.3, 5.0, 2.0, 0.0),
        door('living_room', 'kitchen', 2, 4.3, 5.0, 4.0, 0.0),
        door('kitchen', 'living_room', 1, 6.7, 6.0, 2.0, 3.14159),
        door('kitchen', 'living_room', 2, 6.7, 6.0, 4.0, 3.14159),
    ]
    waypoints = _route_waypoints(
        grid,
        (2.0, 4.0, 0.0),
        (8.0, 4.0, 0.0),
        [(6.0, 1.0), (10.0, 1.0), (10.0, 5.0), (6.0, 5.0)],
        doorways=doorways,
        from_room='living_room',
        to_room='kitchen',
    )

    assert len(waypoints) == 2
    assert all(abs(pose[1] - 4.0) < 0.11 for pose in waypoints)


def test_adjacent_rooms_use_one_shared_doorway_without_backtracking(tmp_path):
    _write_export(tmp_path)
    grid = OccupancyMap(tmp_path / 'LayoutA' / 'map.yaml')
    doorways = [{
        'id': 'living_room_entrance_1',
        'room': 'living_room',
        'connected_room': 'kitchen',
        'width': 1.0,
        'center': {'x': 5.5, 'y': 4.0, 'yaw': 0.0},
        'inside_pose': {'x': 4.8, 'y': 4.0, 'yaw': 0.0},
        'outside_pose': {'x': 6.2, 'y': 4.0, 'yaw': 0.0},
    }, {
        'id': 'kitchen_entrance_1',
        'room': 'kitchen',
        'connected_room': 'living_room',
        'width': 1.0,
        'center': {'x': 5.5, 'y': 4.0, 'yaw': 3.14159},
        'outside_pose': {'x': 4.8, 'y': 4.0, 'yaw': 3.14159},
        'inside_pose': {'x': 6.2, 'y': 4.0, 'yaw': 3.14159},
    }]

    waypoints = _route_waypoints(
        grid,
        (2.0, 4.0, 0.0),
        (8.0, 4.0, 0.0),
        [(6.0, 1.0), (10.0, 1.0), (10.0, 5.0), (6.0, 5.0)],
        doorways=doorways,
        from_room='living_room',
        to_room='kitchen',
    )

    assert len(waypoints) == 2
    assert waypoints[0][0] < waypoints[1][0]


def test_import_generates_package_maps_and_navigation_poses(tmp_path):
    export_root = tmp_path / 'export'
    package_root = tmp_path / 'handyman_rebuild_ros2'
    package_root.mkdir()
    (package_root / 'package.xml').write_text('<package/>', encoding='utf-8')
    _write_export(export_root)

    report = import_layouts(export_root, package_root, ('LayoutA',))

    generated = (
        package_root / 'config' / 'environments' / 'layout_a.yaml'
    ).read_text(encoding='utf-8')
    assert 'internal_name: LayoutA' in generated
    assert 'package://handyman_rebuild_ros2/maps/LayoutA/map.yaml' in generated
    assert generated.count('search_points:') == 4
    assert 'routes:' in generated
    assert 'room: living_room' in generated
    assert (package_root / 'maps' / 'LayoutA' / 'map.pgm').is_file()
    layout_report = report['layouts'][0]
    assert layout_report['rooms_disconnected_from_robot'] == []
    assert layout_report['destination_approach_counts'][
        'table#living_room'
    ] >= 2
    assert layout_report['route_waypoint_counts']

import json

import pytest

from handyman_map_tools.semantic_map import (
    SemanticMapError,
    load,
    point_xy,
    validate,
)


def valid_document():
    rooms = []
    for room_id in ('living_room', 'kitchen', 'bedroom', 'lobby'):
        rooms.append({
            'id': room_id,
            'polygon': [
                {'x': 0.0, 'y': 0.0},
                {'x': 1.0, 'y': 0.0},
                {'x': 1.0, 'y': 1.0},
            ],
        })
    return {
        'schema_version': 1,
        'environment': 'LayoutA',
        'coordinate_system': 'ros_map',
        'rooms': rooms,
        'destinations': [{
            'id': 'table#living_room',
            'room': 'living_room',
            'pose': {'x': 0.5, 'y': 0.5, 'yaw': 0.0},
        }],
        'object_spawn_candidates': [{
            'id': 'candidate01',
            'room': 'living_room',
            'pose': {'x': 0.5, 'y': 0.5, 'yaw': 0.0},
        }],
        'robot_initial_pose': {'x': 0.1, 'y': 0.1, 'yaw': 0.0},
        'messages': [],
    }


def test_valid_document_passes():
    result = validate(valid_document())
    assert result.ok
    assert result.errors == ()


def test_missing_room_fails():
    document = valid_document()
    document['rooms'].pop()
    result = validate(document)
    assert not result.ok
    assert any('missing expected rooms' in item for item in result.errors)


def test_duplicate_destination_fails():
    document = valid_document()
    document['destinations'].append(document['destinations'][0].copy())
    result = validate(document)
    assert any('duplicate destination id' in item for item in result.errors)


def test_unity_error_is_blocking():
    document = valid_document()
    document['messages'].append({'severity': 'error', 'text': 'missing area'})
    result = validate(document)
    assert any(
        'Unity exporter: missing area' in item for item in result.errors
    )


def test_point_rejects_non_finite_value():
    with pytest.raises(SemanticMapError):
        point_xy({'x': float('nan'), 'y': 1.0})


def test_load_accepts_utf8_bom(tmp_path):
    path = tmp_path / 'semantic_map.json'
    path.write_text('\ufeff' + json.dumps(valid_document()), encoding='utf-8')
    assert load(str(path))['environment'] == 'LayoutA'


def test_schema_v2_doorway_and_model_geometry_pass():
    document = valid_document()
    document['schema_version'] = 2
    document['doorways'] = [{
        'id': 'kitchen_entrance_1',
        'room': 'kitchen',
        'connected_room': '',
        'width': 0.9,
        'center': {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
        'outside_pose': {'x': 0.3, 'y': 2.0, 'yaw': 0.0},
        'inside_pose': {'x': 1.7, 'y': 2.0, 'yaw': 0.0},
    }]
    document['destinations'][0].update({
        'model_pose': {'x': 0.5, 'y': 0.5, 'yaw': 1.0},
        'front_yaw': 1.0,
        'front_source': 'model_transform_forward',
    })

    assert validate(document).ok

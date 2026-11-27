"""Loading and validation for the Unity semantic-map interchange format."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple


SUPPORTED_SCHEMA_VERSIONS = {1, 2}
EXPECTED_ROOMS = {'living_room', 'kitchen', 'bedroom', 'lobby'}


class SemanticMapError(ValueError):
    """Raised when an exported semantic map is unsafe to consume."""


@dataclass(frozen=True)
class ValidationResult:
    """Structured result returned by :func:`validate`."""

    errors: Tuple[str, ...]
    warnings: Tuple[str, ...]

    @property
    def ok(self) -> bool:
        """Return whether the map has no blocking validation errors."""
        return not self.errors


def load(path: str) -> Dict[str, Any]:
    """Load a UTF-8 semantic-map JSON document."""
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise SemanticMapError(f'semantic map does not exist: {source}')
    try:
        with source.open('r', encoding='utf-8-sig') as stream:
            document = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise SemanticMapError(
            f'cannot read semantic map {source}: {error}'
        ) from error
    if not isinstance(document, dict):
        raise SemanticMapError('semantic map root must be a JSON object')
    return document


def point_xy(value: Any, label: str = 'point') -> Tuple[float, float]:
    """Convert a Unity-exported point object or pair to finite floats."""
    if isinstance(value, dict):
        raw_x = value.get('x')
        raw_y = value.get('y')
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        if len(value) != 2:
            raise SemanticMapError(f'{label} must contain exactly two values')
        raw_x, raw_y = value
    else:
        raise SemanticMapError(
            f'{label} must be an object or a two-value list'
        )
    try:
        x = float(raw_x)
        y = float(raw_y)
    except (TypeError, ValueError) as error:
        raise SemanticMapError(
            f'{label} contains a non-numeric value'
        ) from error
    if not math.isfinite(x) or not math.isfinite(y):
        raise SemanticMapError(f'{label} contains a non-finite value')
    return x, y


def pose_xy_yaw(value: Any, label: str = 'pose') -> Tuple[float, float, float]:
    """Convert a pose object to finite x, y, and yaw values."""
    if not isinstance(value, dict):
        raise SemanticMapError(f'{label} must be an object')
    x, y = point_xy(value, label)
    try:
        yaw = float(value.get('yaw'))
    except (TypeError, ValueError) as error:
        raise SemanticMapError(f'{label}.yaw is not numeric') from error
    if not math.isfinite(yaw):
        raise SemanticMapError(f'{label}.yaw is not finite')
    return x, y, yaw


def validate(document: Dict[str, Any]) -> ValidationResult:
    """Validate required structure, identifiers, and numeric geometry."""
    errors: List[str] = []
    warnings: List[str] = []

    if document.get('schema_version') not in SUPPORTED_SCHEMA_VERSIONS:
        errors.append(
            'unsupported schema_version: '
            f"{document.get('schema_version')!r}; expected one of "
            f'{sorted(SUPPORTED_SCHEMA_VERSIONS)}'
        )
    if not _nonempty_string(document.get('environment')):
        errors.append('environment must be a non-empty string')
    if document.get('coordinate_system') != 'ros_map':
        errors.append('coordinate_system must be "ros_map"')

    rooms = _list_field(document, 'rooms', errors)
    destinations = _list_field(document, 'destinations', errors)
    candidates = _list_field(document, 'object_spawn_candidates', errors)
    doorways = document.get('doorways', [])
    if not isinstance(doorways, list):
        errors.append('doorways must be a list')
        doorways = []

    room_ids = _unique_ids(rooms, 'room', errors)
    missing_rooms = sorted(EXPECTED_ROOMS - set(room_ids))
    if missing_rooms:
        errors.append('missing expected rooms: ' + ', '.join(missing_rooms))

    for index, room in enumerate(rooms):
        polygon = room.get('polygon') if isinstance(room, dict) else None
        if not isinstance(polygon, list) or len(polygon) < 3:
            errors.append(
                f'rooms[{index}].polygon must have at least 3 points'
            )
            continue
        for point_index, point in enumerate(polygon):
            _check_point(
                point,
                f'rooms[{index}].polygon[{point_index}]',
                errors,
            )

    doorway_ids = _unique_ids(doorways, 'doorway', errors)
    if document.get('schema_version') == 2 and not doorway_ids:
        warnings.append('schema v2 map contains no detected doorways')
    for index, doorway in enumerate(doorways):
        if not isinstance(doorway, dict):
            continue
        room = doorway.get('room')
        if room not in room_ids:
            errors.append(
                f'doorways[{index}] refers to unknown room {room!r}'
            )
        for field in ('center', 'outside_pose', 'inside_pose'):
            _check_pose(
                doorway.get(field),
                f'doorways[{index}].{field}',
                errors,
            )
        try:
            width = float(doorway.get('width'))
            if not math.isfinite(width) or width <= 0.0:
                raise ValueError
        except (TypeError, ValueError):
            errors.append(f'doorways[{index}].width must be positive')

    destination_ids = _unique_ids(destinations, 'destination', errors)
    if not destination_ids:
        errors.append('at least one destination is required')
    for index, destination in enumerate(destinations):
        if not isinstance(destination, dict):
            continue
        _check_pose(
            destination.get('pose'),
            f'destinations[{index}].pose',
            errors,
        )
        if document.get('schema_version') == 2:
            model_pose = destination.get('model_pose')
            if model_pose is not None:
                _check_pose(
                    model_pose,
                    f'destinations[{index}].model_pose',
                    errors,
                )
            if destination.get('front_source') == 'destination_marker_fallback':
                warnings.append(
                    f'destination {destination.get("id", index)!r} has no '
                    'matched rendered model'
                )
        room = destination.get('room', '')
        if room and room not in room_ids:
            warnings.append(
                f'destination {destination.get("id", index)!r} refers to '
                f'unknown room {room!r}'
            )

    candidate_ids = _unique_ids(candidates, 'spawn candidate', errors)
    if not candidate_ids:
        errors.append('at least one object spawn candidate is required')
    for index, candidate in enumerate(candidates):
        if not isinstance(candidate, dict):
            continue
        _check_pose(
            candidate.get('pose'),
            f'object_spawn_candidates[{index}].pose',
            errors,
        )
        room = candidate.get('room', '')
        if not room:
            warnings.append(
                f'spawn candidate {candidate.get("id", index)!r} is outside '
                'all exported room polygons'
            )
        elif room not in room_ids:
            errors.append(
                f'spawn candidate {candidate.get("id", index)!r} refers to '
                f'unknown room {room!r}'
            )

    robot_pose = document.get('robot_initial_pose')
    if robot_pose is None:
        warnings.append('robot_initial_pose was not exported')
    else:
        _check_pose(robot_pose, 'robot_initial_pose', errors)

    for message in document.get('messages', []):
        if not isinstance(message, dict):
            warnings.append(
                'Unity exporter returned a malformed message entry'
            )
            continue
        text = str(message.get('text', 'unspecified Unity exporter message'))
        if message.get('severity') == 'error':
            errors.append('Unity exporter: ' + text)
        else:
            warnings.append('Unity exporter: ' + text)

    return ValidationResult(tuple(errors), tuple(warnings))


def require_valid(document: Dict[str, Any]) -> ValidationResult:
    """Validate a map and raise one aggregate exception on failure."""
    result = validate(document)
    if result.errors:
        raise SemanticMapError('; '.join(result.errors))
    return result


def _list_field(
    document: Dict[str, Any], field: str, errors: List[str]
) -> List[Any]:
    value = document.get(field)
    if not isinstance(value, list):
        errors.append(f'{field} must be a list')
        return []
    return value


def _unique_ids(
    values: Iterable[Any], label: str, errors: List[str]
) -> List[str]:
    identifiers: List[str] = []
    for index, value in enumerate(values):
        if not isinstance(value, dict):
            errors.append(f'{label}[{index}] must be an object')
            continue
        identifier = value.get('id')
        if not _nonempty_string(identifier):
            errors.append(f'{label}[{index}].id must be a non-empty string')
            continue
        if identifier in identifiers:
            errors.append(f'duplicate {label} id: {identifier}')
        identifiers.append(identifier)
    return identifiers


def _check_point(value: Any, label: str, errors: List[str]) -> None:
    try:
        point_xy(value, label)
    except SemanticMapError as error:
        errors.append(str(error))


def _check_pose(value: Any, label: str, errors: List[str]) -> None:
    try:
        pose_xy_yaw(value, label)
    except SemanticMapError as error:
        errors.append(str(error))


def _nonempty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())

"""Small dependency-free helpers for exported ROS occupancy grids."""

from collections import deque
import heapq
import math
from pathlib import Path
import re
from typing import Iterable, List, Sequence, Tuple


FREE = 254


class OccupancyMapError(ValueError):
    """Raised when an exported map cannot be safely interpreted."""


class OccupancyMap:
    """Read a Nav2 YAML/PGM pair and expose map-coordinate queries."""

    def __init__(self, yaml_path: Path):
        self.yaml_path = Path(yaml_path)
        text = self.yaml_path.read_text(encoding='utf-8-sig')
        image = self._field(text, 'image')
        self.resolution = float(self._field(text, 'resolution'))
        origin_match = re.search(
            r'^origin\s*:\s*\[\s*([^,]+),\s*([^,]+),',
            text,
            re.MULTILINE,
        )
        if origin_match is None:
            raise OccupancyMapError('map YAML has no two-dimensional origin')
        self.origin_x = float(origin_match.group(1))
        self.origin_y = float(origin_match.group(2))
        image_path = Path(image)
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path
        self.width, self.height, self.pixels = self._read_pgm(image_path)
        expected = self.width * self.height
        if len(self.pixels) != expected:
            raise OccupancyMapError(
                f'PGM payload has {len(self.pixels)} bytes; expected {expected}'
            )
        self._clearance_cells = self._build_clearance()
        self._component_labels, self.component_sizes = (
            self._build_components()
        )

    @staticmethod
    def _field(text: str, name: str) -> str:
        match = re.search(
            rf'^{re.escape(name)}\s*:\s*([^#\r\n]+)',
            text,
            re.MULTILINE,
        )
        if match is None:
            raise OccupancyMapError(f'map YAML has no {name!r} field')
        return match.group(1).strip().strip('"\'')

    @staticmethod
    def _read_pgm(path: Path) -> Tuple[int, int, bytes]:
        raw = path.read_bytes()
        position = 0
        tokens: List[bytes] = []
        while len(tokens) < 4:
            while position < len(raw) and raw[position:position + 1].isspace():
                position += 1
            if position >= len(raw):
                raise OccupancyMapError('incomplete PGM header')
            if raw[position:position + 1] == b'#':
                newline = raw.find(b'\n', position)
                if newline < 0:
                    raise OccupancyMapError('unterminated PGM comment')
                position = newline + 1
                continue
            end = position
            while end < len(raw) and not raw[end:end + 1].isspace():
                end += 1
            tokens.append(raw[position:end])
            position = end
        if tokens[0] != b'P5' or tokens[3] != b'255':
            raise OccupancyMapError('only 8-bit binary PGM (P5) is supported')
        while position < len(raw) and raw[position:position + 1].isspace():
            position += 1
        return int(tokens[1]), int(tokens[2]), raw[position:]

    def _build_clearance(self) -> List[int]:
        """Return conservative Manhattan clearance from non-free cells."""
        count = self.width * self.height
        distances = [-1] * count
        pending = deque()
        for index, value in enumerate(self.pixels):
            if value != FREE:
                distances[index] = 0
                pending.append(index)
        while pending:
            current = pending.popleft()
            for neighbor in self.neighbors(current):
                if distances[neighbor] < 0:
                    distances[neighbor] = distances[current] + 1
                    pending.append(neighbor)
        return distances

    def _build_components(self) -> Tuple[List[int], List[int]]:
        labels = [-1] * (self.width * self.height)
        sizes: List[int] = []
        for start, value in enumerate(self.pixels):
            if value != FREE or labels[start] >= 0:
                continue
            component = len(sizes)
            pending = deque([start])
            labels[start] = component
            size = 0
            while pending:
                current = pending.popleft()
                size += 1
                for neighbor in self.neighbors(current):
                    if (
                        self.pixels[neighbor] == FREE
                        and labels[neighbor] < 0
                    ):
                        labels[neighbor] = component
                        pending.append(neighbor)
            sizes.append(size)
        return labels, sizes

    def neighbors(self, index: int) -> Iterable[int]:
        column = index % self.width
        if column > 0:
            yield index - 1
        if column + 1 < self.width:
            yield index + 1
        if index >= self.width:
            yield index - self.width
        if index + self.width < len(self.pixels):
            yield index + self.width

    def world_to_index(self, x: float, y: float) -> int:
        column = math.floor((x - self.origin_x) / self.resolution)
        grid_y = math.floor((y - self.origin_y) / self.resolution)
        row = self.height - 1 - grid_y
        if not (0 <= column < self.width and 0 <= row < self.height):
            return -1
        return row * self.width + column

    def index_to_world(self, index: int) -> Tuple[float, float]:
        column = index % self.width
        row = index // self.width
        grid_y = self.height - 1 - row
        return (
            self.origin_x + (column + 0.5) * self.resolution,
            self.origin_y + (grid_y + 0.5) * self.resolution,
        )

    def is_free(self, index: int) -> bool:
        return 0 <= index < len(self.pixels) and self.pixels[index] == FREE

    def clearance(self, index: int) -> float:
        if not self.is_free(index):
            return 0.0
        return self._clearance_cells[index] * self.resolution

    def component(self, x: float, y: float) -> int:
        index = self.world_to_index(x, y)
        return self._component_labels[index] if index >= 0 else -1

    def shortest_safe_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        minimum_clearance: float = 0.30,
    ) -> List[int]:
        """Return a four-connected path biased toward high-clearance space."""
        start_index = self.world_to_index(*start)
        goal_index = self.world_to_index(*goal)
        if (
            self.clearance(start_index) < minimum_clearance
            or self.clearance(goal_index) < minimum_clearance
        ):
            return []
        previous = {start_index: -1}
        costs = {start_index: 0.0}
        goal_column = goal_index % self.width
        goal_row = goal_index // self.width
        pending = [(0.0, start_index)]
        while pending:
            _, current = heapq.heappop(pending)
            if current == goal_index:
                break
            for neighbor in self.neighbors(current):
                clearance = self.clearance(neighbor)
                if clearance < minimum_clearance:
                    continue
                # Penalize cells close to walls. A pure breadth-first path is
                # shortest but tends to scrape door jambs and corridor edges.
                new_cost = costs[current] + 1.0 + 0.8 / (clearance + 0.05)
                if neighbor not in costs or new_cost < costs[neighbor]:
                    costs[neighbor] = new_cost
                    previous[neighbor] = current
                    column = neighbor % self.width
                    row = neighbor // self.width
                    heuristic = abs(column - goal_column) + abs(row - goal_row)
                    heapq.heappush(pending, (new_cost + heuristic, neighbor))
        if goal_index not in previous:
            return []
        result = []
        current = goal_index
        while current >= 0:
            result.append(current)
            current = previous[current]
        result.reverse()
        return result

    def safe_cells_in_polygon(
        self,
        polygon: Sequence[Tuple[float, float]],
        minimum_clearance: float,
        sample_step: float = 0.20,
    ) -> List[int]:
        stride = max(1, round(sample_step / self.resolution))
        result = []
        for row in range(0, self.height, stride):
            for column in range(0, self.width, stride):
                index = row * self.width + column
                if self.clearance(index) < minimum_clearance:
                    continue
                if point_in_polygon(self.index_to_world(index), polygon):
                    result.append(index)
        return result


def point_in_polygon(
    point: Tuple[float, float],
    polygon: Sequence[Tuple[float, float]],
) -> bool:
    """Return whether point is inside an arbitrary simple polygon."""
    x, y = point
    inside = False
    previous = len(polygon) - 1
    for current, a in enumerate(polygon):
        b = polygon[previous]
        if (a[1] > y) != (b[1] > y):
            crossing_x = (
                (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
            )
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside

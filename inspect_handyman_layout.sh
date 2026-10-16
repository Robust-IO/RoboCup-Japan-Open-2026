#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 A|B|C|D [export-root]" >&2
  exit 2
fi

case "${1}" in
  A|a|LayoutA|layout_a) LAYOUT="LayoutA" ;;
  B|b|LayoutB|layout_b) LAYOUT="LayoutB" ;;
  C|c|LayoutC|layout_c) LAYOUT="LayoutC" ;;
  D|d|LayoutD|layout_d) LAYOUT="LayoutD" ;;
  *) echo "ERROR: layout must be A, B, C, or D" >&2; exit 2 ;;
esac

EXPORT_ROOT="${2:-/mnt/d/handyman-export}"
MAP_YAML="${EXPORT_ROOT}/${LAYOUT}/map.yaml"
SEMANTIC_JSON="${EXPORT_ROOT}/${LAYOUT}/semantic_map.json"
if [[ ! -f "${MAP_YAML}" || ! -f "${SEMANTIC_JSON}" ]]; then
  echo "ERROR: ${LAYOUT} export is incomplete under ${EXPORT_ROOT}" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  source "${SCRIPT_DIR}/install/setup.bash"
fi
set -u

if ! ros2 node list 2>/dev/null | grep -qx '/map_server'; then
  nohup ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="${MAP_YAML}" \
    > /tmp/handyman_map_server.log 2>&1 &
  for _ in {1..20}; do
    if ros2 lifecycle get /map_server >/dev/null 2>&1; then
      break
    fi
    sleep 0.25
  done
fi
if ! ros2 lifecycle get /map_server >/dev/null 2>&1; then
  echo "ERROR: map_server did not start; see /tmp/handyman_map_server.log" >&2
  exit 1
fi

MARKER_STARTED=false
if ! ros2 node list 2>/dev/null | grep -qx '/handyman_semantic_marker'; then
  nohup ros2 run handyman_map_tools semantic_marker_node --ros-args \
    -p input:="${SEMANTIC_JSON}" -p frame_id:=map \
    > /tmp/handyman_semantic_marker.log 2>&1 &
  MARKER_STARTED=true
  for _ in {1..20}; do
    if ros2 node list 2>/dev/null \
      | grep -qx '/handyman_semantic_marker'; then
      break
    fi
    sleep 0.25
  done
fi
if ! ros2 node list 2>/dev/null | grep -qx '/handyman_semantic_marker'; then
  echo "ERROR: semantic marker did not start; see /tmp/handyman_semantic_marker.log" >&2
  exit 1
fi

STATE="$(ros2 lifecycle get /map_server | awk '{print $1}')"
if [[ "${STATE}" == "active" ]]; then
  ros2 lifecycle set /map_server deactivate
  STATE="inactive"
fi
if [[ "${STATE}" == "inactive" ]]; then
  ros2 lifecycle set /map_server cleanup
  STATE="unconfigured"
fi
if [[ "${STATE}" != "unconfigured" ]]; then
  echo "ERROR: unsupported map_server lifecycle state: ${STATE}" >&2
  exit 1
fi

ros2 param set /map_server yaml_filename "${MAP_YAML}"
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
if [[ "${MARKER_STARTED}" == false ]]; then
  ros2 param set /handyman_semantic_marker input "${SEMANTIC_JSON}"
fi

echo "RViz switched to ${LAYOUT}"
echo "Map: ${MAP_YAML}"
echo "If the map is outside the current view, use the RViz Views panel to reset the view."

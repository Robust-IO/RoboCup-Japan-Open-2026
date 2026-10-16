#!/usr/bin/env bash
set -eo pipefail

EXPORT_ROOT="${1:-/mnt/d/handyman-export}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${2:-${SCRIPT_DIR}}"
PACKAGE_ROOT="${WORKSPACE_ROOT}/src/handyman_rebuild_ros2"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble was not found in /opt/ros/humble" >&2
  exit 1
fi
if [[ ! -d "${EXPORT_ROOT}/LayoutA" ]]; then
  echo "ERROR: Unity export root is invalid: ${EXPORT_ROOT}" >&2
  exit 1
fi
if [[ ! -f "${PACKAGE_ROOT}/package.xml" ]]; then
  echo "ERROR: handyman_rebuild_ros2 was not found under ${WORKSPACE_ROOT}/src" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi
set -u

echo "[1/4] Importing LayoutA-D from ${EXPORT_ROOT}"
ros2 run handyman_map_tools import_unity_maps \
  "${EXPORT_ROOT}" "${PACKAGE_ROOT}"

echo "[2/4] Installing generated maps and configuration"
cd "${WORKSPACE_ROOT}"
colcon build --base-paths src --symlink-install \
  --packages-select handyman_map_tools handyman_rebuild_ros2
set +u
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

echo "[3/4] Running map, configuration, parser, and protocol tests"
colcon test --base-paths src \
  --packages-select handyman_map_tools handyman_rebuild_ros2 \
  --return-code-on-test-failure

echo "[4/4] Validating Unity semantic exports"
for semantic_map in "${EXPORT_ROOT}"/Layout{A,B,C,D}/semantic_map.json; do
  ros2 run handyman_map_tools validate_semantic_map "${semantic_map}"
done

echo "HANDYMAN MAP IMPORT PASSED"
echo "Report: ${PACKAGE_ROOT}/maps/import_report.json"

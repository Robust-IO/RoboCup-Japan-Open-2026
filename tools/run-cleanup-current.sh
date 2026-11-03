#!/usr/bin/env bash
# Run the updated workspace without inheriting the historical ROS overlay.
set -eo pipefail
cleanup_ws=/home/myw77/RoboCup-Japan-Open-2026
cleanup_python=/home/myw77/RoboCup-Japan-Open-2026/.pixi/envs/default/bin
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH PYTHONHOME
unset LD_LIBRARY_PATH ROS_PACKAGE_PATH
export PATH="$cleanup_python:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source "$cleanup_ws/install/local_setup.bash"
cd "$cleanup_ws"
python - <<'PY'
from ament_index_python.packages import get_package_prefix
root = '/home/myw77/RoboCup-Japan-Open-2026/install/'
for package in ('interactive_cleanup', 'cleanup_vision_ros2',
                'interactive_cleanup_msgs', 'sigverse_ros_bridge'):
    prefix = get_package_prefix(package)
    if not prefix.startswith(root):
        raise SystemExit(f'Wrong workspace for {package}: {prefix}')
    print(f'{package}: {prefix}')
from cleanup_vision_ros2.msg import PointingDirection
assert {'arm_ray_valid', 'geometry_source'} <= PointingDirection.get_fields_and_field_types().keys(), 'Rebuild cleanup_vision_ros2 first'
PY
echo "Working tree base (with local fixes): $(git rev-parse HEAD)"
echo "Installed controller / detector / bridge fingerprints:"
sha256sum install/interactive_cleanup/lib/interactive_cleanup/interactive_cleanup_sample \
  install/cleanup_vision_ros2/lib/cleanup_vision_ros2/cleanup_detection_node \
  install/sigverse_ros_bridge/lib/sigverse_ros_bridge/sigverse_ros_bridge
if [[ "${1:-}" == --check ]]; then
    exec ros2 launch interactive_cleanup cleanup.launch.py --show-args
fi
mkdir -p "$cleanup_ws/test-logs"
cleanup_log="$cleanup_ws/test-logs/$(date +%Y%m%d-%H%M%S).log"
echo "Console log: $cleanup_log"
ros2 launch interactive_cleanup cleanup.launch.py use_nav2:=false use_rviz:=false "$@" 2>&1 | tee "$cleanup_log"

#!/usr/bin/env bash
set -eo pipefail
cleanup_ws=/home/myw77/RoboCup-Japan-Open-2026
cleanup_install=/home/myw77/cleanup-lowpoint-run/install
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH ROS_PACKAGE_PATH
export PATH="$cleanup_ws/.pixi/envs/default/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$cleanup_install/local_setup.bash"
cd "$cleanup_ws"
python - <<'PY'
from ament_index_python.packages import get_package_prefix
root = '/home/myw77/cleanup-lowpoint-run/install/'
for package in ('interactive_cleanup', 'cleanup_vision_ros2', 'interactive_cleanup_msgs', 'sigverse_ros_bridge'):
    prefix = get_package_prefix(package)
    if not prefix.startswith(root):
        raise SystemExit(f'Wrong workspace for {package}: {prefix}')
    print(f'{package}: {prefix}')
PY
echo "Source base: $(git rev-parse HEAD) (local modifications may be present)"
sha256sum "$cleanup_install/interactive_cleanup/lib/interactive_cleanup/interactive_cleanup_sample" "$cleanup_install/cleanup_vision_ros2/share/cleanup_vision_ros2/models/cleanup_model.pt"
if [[ "${1:-}" == --check ]]; then
    exec ros2 launch interactive_cleanup cleanup.launch.py --show-args
fi
mkdir -p /home/myw77/cleanup-lowpoint-run/test-logs
cleanup_log="/home/myw77/cleanup-lowpoint-run/test-logs/$(date +%Y%m%d-%H%M%S).log"
echo "Console log: $cleanup_log"
ros2 launch interactive_cleanup cleanup.launch.py use_nav2:=false use_rviz:=false "$@" 2>&1 | tee "$cleanup_log"

#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
mkdir -p /tmp/rviz-runtime-local
chmod 700 /tmp/rviz-runtime-local
export XDG_RUNTIME_DIR=/tmp/rviz-runtime-local
export DISPLAY=:0 QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1
export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/dual-camera-rviz-logs
exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py \
  --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml \
  --run -- rviz2 -d /__HANDYMAN_PACKAGE__/project/training/config/live_head_hand_rgb.rviz --geometry 1200x760+60+60

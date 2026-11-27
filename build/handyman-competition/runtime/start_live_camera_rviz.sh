#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
mkdir -p /tmp/rviz-runtime-local
chmod 700 /tmp/rviz-runtime-local
export XDG_RUNTIME_DIR=/tmp/rviz-runtime-local
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
export ROS_LOG_DIR=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/gripper-rviz-logs
exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py \
  --domain 71 \
  --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml \
  --run -- rviz2 -d /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/live_head_rgb.rviz --geometry 800x560+20+20

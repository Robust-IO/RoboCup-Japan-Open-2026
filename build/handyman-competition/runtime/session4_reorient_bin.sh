#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash
export PYTHONPATH=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/workspace/training/session1:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/generic-grasp-v1:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1:${PYTHONPATH:-}
exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/session4_reorient_bin.py "$@"

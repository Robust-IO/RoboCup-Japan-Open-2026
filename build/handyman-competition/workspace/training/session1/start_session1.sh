#!/usr/bin/env bash
set -eo pipefail
SESSION1_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SESSION1_PROJECT=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project
source /opt/ros/humble/setup.bash
source "$SESSION1_PROJECT/install/setup.bash"
set -u
export PYTHONPATH="$SESSION1_ROOT:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/generic-grasp-v1:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1:${PYTHONPATH:-}"
exec /usr/bin/python3 "$SESSION1_PROJECT/training/scripts/cyclone_test_env.py" \
  --domain 71 --dds-config "$SESSION1_PROJECT/training/config/cyclone_search_stack.xml" \
  --run -- /usr/bin/python3 "$SESSION1_ROOT/run_live_session1.py" \
  --config "$SESSION1_ROOT/session1-live-config.json" "$@"

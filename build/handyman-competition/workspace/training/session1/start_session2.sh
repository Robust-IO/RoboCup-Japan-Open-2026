#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project
source /opt/ros/humble/setup.bash
source "$PROJECT/install/setup.bash"
export PYTHONPATH="$ROOT:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/generic-grasp-v1:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1:${PYTHONPATH:-}"
exec /usr/bin/python3 "$PROJECT/training/scripts/cyclone_test_env.py" --domain 71 --dds-config "$PROJECT/training/config/cyclone_search_stack.xml" --run -- /usr/bin/python3 "$ROOT/run_live_session1.py" --config "$ROOT/session2-live-config.json" "$@"

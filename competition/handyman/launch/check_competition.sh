#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export HANDYMAN_COMPETITION_ROOT="$ROOT"
/usr/bin/python3 "$ROOT/resolve_package_paths.py"
source /opt/ros/humble/setup.bash
source "$ROOT/project/install/setup.bash"
export PYTHONPATH="$ROOT/workspace/training/session1:$ROOT/project/training/scripts:$ROOT/runtime/generic-grasp-v1:$ROOT/runtime:$ROOT/tools/apple-transfer-auto-v1:${PYTHONPATH:-}"
exec /usr/bin/python3 "$ROOT/project/training/scripts/cyclone_test_env.py" --domain 71 --dds-config "$ROOT/project/training/config/cyclone_search_stack.xml" --run -- /usr/bin/python3 "$ROOT/check_package.py"

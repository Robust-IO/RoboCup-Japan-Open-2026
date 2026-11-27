#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT=/__HANDYMAN_PACKAGE__/project
source /opt/ros/humble/setup.bash
source "$PROJECT/install/setup.bash"
export PYTHONPATH="$ROOT:$PROJECT/training/scripts:/__HANDYMAN_PACKAGE__/runtime/generic-grasp-v1:/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:${PYTHONPATH:-}"
exec /usr/bin/python3 "$PROJECT/training/scripts/cyclone_test_env.py" --domain 71 --dds-config "$PROJECT/training/config/cyclone_search_stack.xml" --run -- /usr/bin/python3 "$ROOT/start_session6_round.py" "$@"

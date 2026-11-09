#!/usr/bin/env bash
set -eo pipefail
transfer_code="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
transfer_repo=/home/crazylearner/RoboCup-Japan-Open-2026
source /opt/ros/humble/setup.bash
source "${transfer_repo}/install/setup.bash"
set -u
export HANDYMAN_RESIDENT_MODEL="${HANDYMAN_RESIDENT_MODEL:-1}"
export ROS_LOG_DIR="/mnt/d/handyman-training-runs/object-transfer-roslogs/$(date -u +%Y%m%dT%H%M%S-%N)"
exec /usr/bin/python3 "${transfer_repo}/training/scripts/cyclone_test_env.py" \
  --domain 71 --dds-config "${transfer_repo}/training/config/cyclone_search_stack.xml" --run -- \
  /usr/bin/python3 "${transfer_code}/run_object_transfer.py" "$@"

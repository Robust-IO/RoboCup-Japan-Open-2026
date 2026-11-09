#!/usr/bin/env bash
set -e
transfer_code_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
transfer_repo=/home/crazylearner/RoboCup-Japan-Open-2026
transfer_live=false
for transfer_argument in "$@"; do
  if [[ "$transfer_argument" == --run ]]; then transfer_live=true; fi
done
if [[ "$transfer_live" != true ]]; then
  exec /usr/bin/python3 "$transfer_code_dir/apple_transfer_automation.py" "$@"
fi
source /opt/ros/humble/setup.bash
source "$transfer_repo/install/setup.bash"
exec /usr/bin/python3 "$transfer_repo/training/scripts/cyclone_test_env.py" \
  --domain 71 --dds-config "$transfer_repo/training/config/cyclone_search_stack.xml" \
  --run -- /usr/bin/python3 "$transfer_code_dir/apple_transfer_automation.py" "$@"

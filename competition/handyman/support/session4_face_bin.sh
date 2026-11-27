#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /__HANDYMAN_PACKAGE__/project/install/setup.bash
export PYTHONPATH=/__HANDYMAN_PACKAGE__/workspace/training/session1:/__HANDYMAN_PACKAGE__/project/training/scripts:/__HANDYMAN_PACKAGE__/runtime/generic-grasp-v1:/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:${PYTHONPATH:-}
exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/session4_face_bin.py "$@"

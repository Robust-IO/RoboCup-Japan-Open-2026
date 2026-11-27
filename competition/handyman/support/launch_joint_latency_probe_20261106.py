import subprocess
cmd='source /opt/ros/humble/setup.bash; export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/joint-latency-roslogs-20261106; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/probe_joint_latency_20261106.py'
raise SystemExit(subprocess.run(['bash','-lc',cmd]).returncode)

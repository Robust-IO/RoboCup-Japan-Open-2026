import subprocess
cmd='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/demo-prime-roslogs-20261106; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/prime_demo_navigation_20261106.py'
raise SystemExit(subprocess.run(['bash','-lc',cmd]).returncode)

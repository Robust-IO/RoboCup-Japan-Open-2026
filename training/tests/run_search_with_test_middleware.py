"""Run integration using an explicitly selected temporary middleware manifest."""
import json
import os
from pathlib import Path
import subprocess
import sys

assert os.environ.get('ROS_DOMAIN_ID')=='73'
manifest=Path(sys.argv[1])
env=dict(os.environ)
settings=json.loads(manifest.read_text())
assert set(settings)=={'RMW_IMPLEMENTATION','LD_LIBRARY_PATH','AMENT_PREFIX_PATH'}
assert settings['RMW_IMPLEMENTATION']=='rmw_cyclonedds_cpp'
env.update(settings)
for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_USE_QOS_FROM_XML'):
    env.pop(key,None)
subprocess.run(['/usr/bin/python3',str(Path(__file__).with_name('run_search_ros_integration.py'))],env=env,check=True,timeout=150)

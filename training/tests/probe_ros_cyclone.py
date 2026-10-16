"""Download/extract comparison middleware to a new temp dir; no apt install.

Requires ROS already sourced. Only child test processes use these libraries.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import json
import sys
import shutil

assert os.environ.get('ROS_DOMAIN_ID')=='73'
root=Path(tempfile.mkdtemp(prefix='handyman-cyclone-comparison-'))
print('TEMP',root,flush=True)
packages=['ros-humble-'+p for p in ('rmw-cyclonedds-cpp','cyclonedds',
    'iceoryx-binding-c','iceoryx-hoofs','iceoryx-posh')]
if len(sys.argv)>1:
    for deb in Path(sys.argv[1]).glob('*.deb'):
        shutil.copy2(deb,root/deb.name)
    if len(list(root.glob('*.deb'))) != 5:
        raise RuntimeError('Expected five verified packages')
else:
    subprocess.run(['apt-get','download',*packages],cwd=root,check=True,timeout=180)
for deb in root.glob('*.deb'):
    subprocess.run(['dpkg-deb','--extract',str(deb),str(root/'unpacked')],check=True)
prefix=root/'unpacked/opt/ros/humble'
env=dict(os.environ,RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
    LD_LIBRARY_PATH=str(prefix/'lib')+':'+str(prefix/'lib/x86_64-linux-gnu')+':'+os.environ.get('LD_LIBRARY_PATH',''),
    AMENT_PREFIX_PATH=str(prefix)+':'+os.environ.get('AMENT_PREFIX_PATH',''))
for name in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_USE_QOS_FROM_XML'):
    env.pop(name,None)
(root/'environment.json').write_text(json.dumps({k:env[k] for k in ('RMW_IMPLEMENTATION','LD_LIBRARY_PATH','AMENT_PREFIX_PATH')},indent=2))
subprocess.run(['/usr/bin/python3','/mnt/c/Users/wpb15/probe_ros_official.py'],env=env,check=True,timeout=25)

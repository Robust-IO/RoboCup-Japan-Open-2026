import subprocess,json,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');name='cigarette-contact-diagnostic-20261101-01'
command=['bash','-lc','source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; '
 'export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; '
 'export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/'+name+'-roslogs; '
 'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 '
 '--dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- '
 '/usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/inspect_current_thin_contact_20261101.py']
with (b/(name+'-console.log')).open('xb') as out:p=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
(b/(name+'-launch.json')).write_text(json.dumps(dict(pid=p.pid,command=command,started_ns=time.time_ns()),indent=2));print(p.pid)

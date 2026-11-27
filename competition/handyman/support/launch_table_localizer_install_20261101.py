import json,subprocess,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');name='table-localizer-install-'+time.strftime('%Y%m%d-%H%M%S')
cmd=['bash','-lc','source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/'+name+'-roslogs; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/install_table_localizer_20261101.py']
with (b/(name+'-console.log')).open('xb') as stream:child=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
r=dict(pid=child.pid,log=str(b/(name+'-console.log')));(b/'table-localizer-install-latest.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))

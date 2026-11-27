import json,subprocess,time,shlex
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');a=json.loads((b/'penguin-contact-resume-adapter-latest.json').read_text());f=Path(a['adapter']);root=b/('penguin-contact-continue-'+time.strftime('%Y%m%d-%H%M%S'))
args=['/usr/bin/python3',str(f/'run_object_transfer.py'),'--object','toy_penguin','--run','--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision','--source-imgsz','960','--task-config',a['task'],'--continue-closed-contact',a['predecessor'],'--output',str(root)]
cmd='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export PYTHONPATH='+str(f)+':/__HANDYMAN_PACKAGE__/runtime:$PYTHONPATH; export ROS_LOG_DIR='+str(root)+'-roslogs; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- '+shlex.join(args)
with root.with_suffix('.log').open('x') as log:p=subprocess.Popen(['bash','-lc',cmd],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
r=dict(pid=p.pid,output=str(root),adapter=str(f),predecessor=a['predecessor']);(b/'penguin-contact-continue-latest.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))

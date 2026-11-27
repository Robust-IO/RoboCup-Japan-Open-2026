"""Isolated source-resolution trial, retaining the deployed shared height core."""
import ast,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
prior=json.loads((b/'penguin-kitchen-live-latest.json').read_text());old=json.loads((Path(prior['output'])/'result.json').read_text())
if (old['status']!='stopped_requires_actual_state_review' or old['stage']!='new_toy_penguin_rgbd'
        or old['close_commands_sent']!=0 or old['release_commands_sent']!=0):raise ValueError('ungrasped_source_observation_failure_required')
stamp=time.strftime('%Y%m%d-%H%M%S');trial=b/('penguin-source-resolution-trial-'+stamp)
shutil.copytree(f,trial,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
source=trial/'pepper_transfer_automation.py';before=source.read_text();text=before
pairs=[
    ("object_name='ground_pepper',navigation_profile=None):", "object_name='ground_pepper',navigation_profile=None,source_imgsz=640):"),
    ("    report['configured_task']=task", "    report['source_detection_imgsz']=source_imgsz\n    report['configured_task']=task"),
    ("runtime.root/'model-worker.log',imgsz=640)", "runtime.root/'model-worker.log',imgsz=source_imgsz)"),
    ("    p.add_argument('--task-config',type=Path)", "    p.add_argument('--task-config',type=Path)\n    p.add_argument('--source-imgsz',type=int,choices=(640,960,1280),default=640)"),
    ("object_name=object_name,navigation_profile=args.navigation_profile)", "object_name=object_name,navigation_profile=args.navigation_profile,source_imgsz=args.source_imgsz)")]
for a,z in pairs:
    if text.count(a)!=1:raise ValueError('source_change_not_unique:'+a)
    text=text.replace(a,z,1)
ast.parse(text);source.write_text(text)
(trial/'pepper_transfer_automation.before.py.txt').write_text(before)
env=dict(os.environ,PYTHONPATH=str(trial)+':'+str(b))
tests=subprocess.run(['/usr/bin/python3','-m','unittest','test_pepper_tasks','test_penguin_geometry','test_penguin_source_identity','test_pepper_transfer','-q'],env=env,cwd='/tmp',capture_output=True,text=True,timeout=120)
(trial/'resolution-trial-tests.log').write_text(tests.stdout+tests.stderr)
if tests.returncode:raise RuntimeError((tests.stdout+tests.stderr)[-5000:])
task=Path(prior['task'])
plan=subprocess.run(['/usr/bin/python3',str(trial/'run_object_transfer.py'),'--object','toy_penguin','--task-config',str(task),'--source-imgsz','960'],env=env,capture_output=True,text=True,check=True)
(trial/'resolution-trial-plan.json').write_text(plan.stdout)
root=b/('penguin-bedroom-kitchen-960-'+stamp)
cmd='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export HANDYMAN_RESIDENT_MODEL=1; export PYTHONPATH='+str(trial)+':/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; export ROS_LOG_DIR='+str(root)+'-roslogs; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(trial/'run_object_transfer.py')+' --object toy_penguin --run --confirm-layout LayoutA --confirm-localization --simulation-ignore-collision --source-imgsz 960 --task-config '+str(task)+' --output '+str(root)
with root.with_suffix('.log').open('x') as log:child=subprocess.Popen(['bash','-lc',cmd],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,output=str(root),task=str(task),started_ns=time.time_ns(),object='toy_penguin',
    source='white_side_table#bedroom_1',destination='dining_table#kitchen',source_imgsz=960,
    source_adapter=str(trial),canonical_sources_changed=False,previous_source_observation_attempt=prior,
    only_source_resolution_config_changed=True,source_before_sha256=hashlib.sha256(before.encode()).hexdigest(),source_after_sha256=hashlib.sha256(text.encode()).hexdigest(),
    offline_test_output=(tests.stdout+tests.stderr)[-1000:])
(trial/'trial-receipt.json').write_text(json.dumps(receipt,indent=2))
(b/'penguin-kitchen-live-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))

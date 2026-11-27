"""Resume verified current hold at kitchen; no pickup or navigation replay."""
import hashlib,json,subprocess,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');adapter_receipt=json.loads((b/'pepper-dynamic-height-adapter-latest.json').read_text());adapter=Path(adapter_receipt['adapter']);previous=Path(adapter_receipt['predecessor']);validation=json.loads((adapter/'recorded-kitchen-replay.json').read_text())
if validation['status']!='validated_recorded_geometry' or not validation['base_reachability']['ready']:raise ValueError('validated_actual_placement_geometry_required')
for row in adapter_receipt['files']:
    if hashlib.sha256((adapter/row['name']).read_bytes()).hexdigest()!=row['sha256']:raise ValueError('adapter_changed_after_validation')
root=b/('pepper-kitchen-dynamic-continue-'+time.strftime('%Y%m%d-%H%M%S'));task=b/'pepper-bedroom-to-kitchen-20261107.json'
cmd='source /opt/ros/humble/setup.bash; source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash; export HANDYMAN_RESIDENT_MODEL=1; export PYTHONPATH='+str(adapter)+':/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime:$PYTHONPATH; export ROS_LOG_DIR='+str(root)+'-roslogs; exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(adapter/'run_object_transfer.py')+' --object ground_pepper --run --confirm-layout LayoutA --confirm-localization --simulation-ignore-collision --task-config '+str(task)+' --continue-destination-hold '+str(previous)+' --output '+str(root)
with root.with_suffix('.log').open('x') as stream:child=subprocess.Popen(['bash','-lc',cmd],stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,output=str(root),predecessor=str(previous),adapter=str(adapter),started_ns=time.time_ns(),grasp_replayed=False,navigation_replayed=False)
(b/'pepper-kitchen-dynamic-continue-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))

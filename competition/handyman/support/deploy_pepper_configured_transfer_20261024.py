"""Validate configured tasks and saved RGBD before atomic code deployment."""
import hashlib,json,os,re,shutil,subprocess,sys,tarfile,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');roots=[code,repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
previous=read(base/'ground-pepper-native-transfer-success-progress-20261024.json')
changed=['pepper_transfer_automation.py','pepper_runtime.py','pepper_place.py']
new=['pepper_tasks.py','pepper_regression.py','test_pepper_tasks.py','test_pepper_regression.py',
    'config/pepper_bedroom_to_living_room.json','config/pepper_living_room_to_bedroom.json']
names=changed+new
for root in roots:
    if any(sha(root/n)!=d for n,d in previous['latest_expected_hashes'].items()):raise ValueError('tested_source_changed')
    if any((root/n).exists() for n in new):raise ValueError('new_files_already_exist')
archive=base/'pepper-configured-transfer-v1-20261024.tar.gz'
if sha(archive)!=sys.argv[1]:raise ValueError('patch_changed')
validation=base/'pepper-configured-transfer-v1-20261024-validation';validation.mkdir(exist_ok=False)
candidate=validation/'candidate';shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(archive) as tar:
    if set(tar.getnames())!=set(names):raise ValueError('unexpected_files')
    for member in tar.getmembers():
        if not member.isfile() or '..' in Path(member.name).parts:raise ValueError('unsafe_archive')
        target=candidate/member.name;target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(tar.extractfile(member).read())
with (validation/'tests.log').open('x') as stream:
    result=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),
        stdout=stream,stderr=subprocess.STDOUT,timeout=180)
log=(validation/'tests.log').read_text();print(log[-1800:],flush=True)
if result.returncode:raise ValueError('regression_failed')
tests=int(re.search(r'Ran (\d+) tests',log).group(1));sys.path[:0]=[str(candidate),str(candidate/'scripts')]
import numpy as np
import yaml
from PIL import Image
from apple_transfer_automation import RobotLock,check_inputs
from pepper_tasks import load_task,configured_transfer,DEFAULT_TASK,REVERSE_TASK
from apple_worker_client import Worker
from pepper_transfer_automation import confirmed_peppers
from pepper_geometry import estimate_source
from inverse_table_place_plan import OFFSET
from apple_transfer_auto_plan import map_point
from can_destination_surface import measure as measure_table
from pepper_place import make_plan,extension_clearance
from scripts.search_room_gate import inside

baseline=read(code/'config/apple_transfer_auto_v1.json');input_audit=check_inputs(baseline)
environment=yaml.safe_load(Path(baseline['environment']).read_text());tables=read(Path(baseline['semantic_map']))['destinations']
forward=configured_transfer(baseline,load_task(DEFAULT_TASK),environment,tables)
reverse=configured_transfer(baseline,load_task(REVERSE_TASK),environment,tables)
for cfg in (forward,reverse):
    for key in ('motion_profile','navigation_tolerances','policy','execution'):
        if cfg[key]!=baseline[key]:raise ValueError('original_motion_policy_changed')
old=read(base/'pepper-native-pregrasp-continuation-20261024-07/result.json')
held=read(base/'pepper-native-pregrasp-continuation-20261024-07/held-context.json')
rows=[];worker=Worker(baseline['model_python'],baseline['registry'],validation/'offline-model-worker.log',imgsz=640)
try:
    for i,capture in enumerate(previous['final_placement_captures']):
        capture=Path(capture);rgb=np.asarray(Image.open(capture/'rgb.png').convert('RGB'));depth=np.load(capture/'depth_m.npy')
        snapshot=read(capture/'snapshot.json');raw=worker.infer(capture)
        (validation/f'reverse-source-{i}-detector.json').write_text(json.dumps(raw,indent=2))
        found=confirmed_peppers(raw.get('detections',[]))
        if len(found)!=1:raise ValueError('saved_living_pepper_not_uniquely_confirmed')
        geometry=estimate_source(rgb,depth,snapshot,found[0]);position=map_point(snapshot,np.asarray(geometry['center_m']))
        if not inside(*position[:2],environment['rooms'][reverse['source_room']]['region']):raise ValueError('saved_reverse_source_wrong_room')
        if not .07<=geometry['center_m'][2]-OFFSET<=.343:raise ValueError('reverse_source_height_outside_operating_range')
        rows.append(dict(capture=str(capture),confidence=found[0]['confidence'],measurement=geometry,
            position_map_m=position.tolist(),historical_observation_only=True,robot_commands_sent=False))
finally:worker.close()
capture=Path(old['source_observations'][-1]['capture']);snapshot=read(capture/'snapshot.json');depth=np.load(capture/'depth_m.npy')
destination=next(t for t in tables if t['id']==reverse['destination_id'])
plane=measure_table(depth,snapshot,destination)
plan=make_plan(depth,snapshot,destination,plane,held,reverse,'offline-white-destination')
audit=dict(status='configured_tasks_and_saved_reverse_source_verified',historical_reference_only=True,
    source_rows=rows,reverse_destination_plane=plane,reverse_destination_plan=plan,
    input_audit=input_audit,robot_commands_sent=False,live_reverse_transfer_verified=False)
(validation/'offline-reference-audit.json').write_text(json.dumps(audit,indent=2,allow_nan=False))
print(json.dumps(dict(offline_source_frames=len(rows),confidences=[r['confidence'] for r in rows],
    white_table_plan_verified=True,robot_commands_sent=False)),flush=True)

with RobotLock(),tarfile.open(validation/'previous-files.tar.gz','x:gz') as backup:
    for root in roots:
        if any(sha(root/n)!=d for n,d in previous['latest_expected_hashes'].items()):raise ValueError('concurrent_edit')
        if any((root/n).exists() for n in new):raise ValueError('concurrent_new_file')
    latest=dict(previous['latest_expected_hashes'])
    for i,root in enumerate(roots):
        for n in changed:backup.add(root/n,arcname=str(i)+'/'+n)
        for n in names:
            pending=root/(n+'.configured-pending');pending.parent.mkdir(parents=True,exist_ok=True)
            with pending.open('xb') as stream:stream.write((candidate/n).read_bytes())
            pending.replace(root/n);latest[n]=sha(candidate/n)
    for root in roots:
        if any(sha(root/n)!=d for n,d in latest.items()):raise ValueError('copy_mismatch')
    checkpoint=dict(status='pepper_configured_bidirectional_tasks_deployed_live_regression_pending',saved_ns=time.time_ns(),
        tests_passed=tests,offline_reverse_source_frames_verified=len(rows),validation_directory=str(validation),
        baseline_input_audit=input_audit,latest_expected_hashes=latest,
        source_object_coordinates_configured=False,navigation_policy_changed=False,
        robot_commands_sent=False,current_robot_state_not_queried=True,
        last_verified_robot_state=previous['last_verified_robot_state'],
        last_robot_state_is_historical=True,live_reverse_transfer_verified=False,
        consecutive_forward_rounds_verified=0,
        next='Read current Unity readiness. If empty/tucked and connected, start automatic reverse reset plus three uninterrupted forward rounds.',
        human_object_reposition_required=False)
    for path in (base/'pepper-configured-transfer-ready-progress-20261024.json',repo/'training/pepper-configured-transfer-ready-progress-20261024.json'):
        with path.open('x') as stream:json.dump(checkpoint,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in checkpoint.items() if k not in ('latest_expected_hashes','last_verified_robot_state','baseline_input_audit')}),flush=True)

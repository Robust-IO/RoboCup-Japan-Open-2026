"""Stage/test only. No ROS imports and no robot action entrypoints."""
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import time

RUNS=Path('/__HANDYMAN_PACKAGE__/runtime')
BASE=json.loads((RUNS/'apple-efficiency-r17-20261022-baseline.json').read_text())
RUNTIME=Path(BASE['roots']['runtime'])
OUT=RUNS/'apple-efficiency-r17-20261022-validation-02'
PATCH=RUNS/'apple-efficiency-r17-20261022-patch.tar.gz'
EXPECTED='a9c15e485a90557c4cd20d921fd79cb27bf6c9a34326b0911f81a5e77bcf9172'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
if OUT.exists():raise RuntimeError('new_validation_output_required')
if sha(PATCH)!=EXPECTED:raise RuntimeError('patch_hash_mismatch')
for rootkey,rootname in BASE['roots'].items():
    for name,expected in BASE['hashes'][rootkey].items():
        if sha(Path(rootname)/name)!=expected:raise RuntimeError('concurrent_code_change:'+rootkey+'/'+name)
OUT.mkdir()
CANDIDATE=OUT/'candidate'
shutil.copytree(RUNTIME,CANDIDATE,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git'))
with tarfile.open(PATCH) as archive:
    if set(archive.getnames())!=set(BASE['files']):raise RuntimeError('patch_file_set_mismatch')
    for member in archive.getmembers():
        if not member.isfile() or Path(member.name).is_absolute() or '..' in Path(member.name).parts:
            raise RuntimeError('unsafe_patch_member')
        path=CANDIDATE/member.name;path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('wb') as stream:stream.write(archive.extractfile(member).read())
for name in BASE['files']:
    if name.endswith('.py'):ast.parse((CANDIDATE/name).read_text(),filename=name)
for name,expected in BASE['frozen']['runtime'].items():
    if sha(CANDIDATE/name)!=expected:raise RuntimeError('frozen_code_changed:'+name)
old_config=json.loads((RUNTIME/'config/apple_transfer_auto_v1.json').read_text())
new_config=json.loads((CANDIDATE/'config/apple_transfer_auto_v1.json').read_text())
comparable=json.loads(json.dumps(new_config));comparable['policy'].pop('efficient_non_navigation')
if comparable!=old_config:raise RuntimeError('unexpected_config_change')
report=dict(revision='efficiency-r17',status='testing',archive_sha256=EXPECTED,
            candidate=str(CANDIDATE),navigation_files_unchanged=True,
            original_recordings_unchanged=True,robot_commands_sent=False,new_live_motion_trial=False)
env=os.environ.copy();env['PYTHONPATH']=str(CANDIDATE)+':'+str(CANDIDATE/'scripts')
def run(name,command):
    with (OUT/(name+'.log')).open('x') as log:
        result=subprocess.run(command,cwd=CANDIDATE,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180)
    text=(OUT/(name+'.log')).read_text()
    print(name,result.returncode,text[-2500:],flush=True)
    if result.returncode:raise RuntimeError(name+'_failed')
    report[name]=dict(exit=0,log=str(OUT/(name+'.log')))
run('all-tests',[sys.executable,'-m','unittest','discover','-s',str(CANDIDATE),'-p','test_*.py'])
run('file-preflight',[sys.executable,str(CANDIDATE/'apple_transfer_automation.py'),'--check-inputs'])

# Recorded source + placed-frame clients share one real model, without Unity.
sys.path.insert(0,str(CANDIDATE))
from apple_shared_inference import SharedWorker
from inspect_table_apple import inspect
from verify_table_placement import verify
OLD=RUNS/'automatic-apple-r16b-20261022-01'
FRAME_ROOT=OUT/'recorded-frame-reuse'
FRAME_ROOT.mkdir()
source=FRAME_ROOT/'source';shutil.copytree(OLD/'source-observe/000181',source)
placed=[]
for name in ('000000','000002','000004'):
    target=FRAME_ROOT/name;shutil.copytree(OLD/'placed-final'/name,target);placed.append(target)
model_log=(OUT/'real-shared-model.log').open('x')
with tempfile.TemporaryDirectory(prefix='hm-r17-test-') as temporary:
    endpoint=Path(temporary)/'worker.sock'
    process=subprocess.Popen([sys.executable,str(CANDIDATE/'apple_shared_inference.py'),
        '--socket',str(endpoint),'--run-root',str(FRAME_ROOT),'--python',new_config['model_python'],
        '--registry',new_config['registry'],'--confirm-apple-roi'],cwd=CANDIDATE,env=env,
        stdout=model_log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        start=time.monotonic()
        first=SharedWorker(endpoint,new_config['registry'],confirm_apple_roi=True)
        cold=time.monotonic()-start
        source_inference=first.infer(source);first.close()
        start=time.monotonic();second=SharedWorker(endpoint,new_config['registry']);warm=time.monotonic()-start
        semantic=json.loads(Path(new_config['semantic_map']).read_text())
        dest=next(x for x in semantic['destinations'] if x['id']==new_config['destination_id'])
        plane=json.loads((OLD/'destination-surface.json').read_text())
        frames=[];durations=[]
        for capture in placed:
            before=time.monotonic();inference=second.infer(capture);durations.append(time.monotonic()-before)
            frame=inspect(capture,inference.get('detections',[]),dest,plane)
            frame['inference']=inference;frames.append(frame)
        second.close()
        actions={key:json.loads((OLD/f'place-{key}/result.json').read_text()) for key in ('release','raise','retract')}
        checked=verify(dict(frames=frames),json.loads((OLD/'pre-release-check.json').read_text()),actions,
            new_config['destination_id'],new_config['destination_room'],
            placement_plan=json.loads((OLD/'placement-plan.json').read_text()))
        if checked['placement_success_verified'] is not True:raise RuntimeError('recorded_final_placement_regression')
        report['real_model_reuse']=dict(service_pid=process.pid,cold_start_s=cold,warm_connect_s=warm,
            final_inference_s=durations,same_process_still_running=process.poll() is None,
            source_status=source_inference['status'],model_sha256=first.ready['model_sha256'],
            recorded_placement_verified=True,recorded_only=True,robot_commands_sent=False)
        with (OUT/'recorded-final-audit.json').open('x') as stream:json.dump(dict(frames=frames,verification=checked),stream,indent=2)
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        model_log.close()
report.update(status='candidate_tests_and_recorded_model_reuse_passed',
              candidate_hashes={name:sha(CANDIDATE/name) for name in BASE['files']})
with (OUT/'result.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report,indent=2),flush=True)

import ast,fcntl,hashlib,json,shutil,subprocess,os,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
lock=Path('/tmp/handyman-apple-transfer-automation.lock').open('a+');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
a=json.loads((b/'penguin-contact-continue-latest.json').read_text());run=Path(a['output']);result=json.loads((run/'result.json').read_text())
assert result['placement_success_verified'] and result['cleanup_completed']
contact=Path(a['adapter']);resolution=Path(json.loads((b/'penguin-contact-resume-adapter-latest.json').read_text())['prior_receipt']['source_adapter'])
updates={f/'pepper_transfer_automation.py':(resolution/'pepper_transfer_automation.py').read_text().replace('source_imgsz=640','source_imgsz=960').replace("choices=(640,960,1280),default=640", "choices=(640,960,1280),default=960"),f/'profiled_capture.py':(contact/'profiled_capture.py').read_text(),b/'profiled_capture.py':(contact/'profiled_capture.py').read_text()}
for v in updates.values():ast.parse(v)
backup=b/'backups'/('before-penguin-camera-resolution-'+time.strftime('%Y%m%d-%H%M%S'));backup.mkdir()
receipt_path=b/'common-placement-deployment-latest.json';receipt=json.loads(receipt_path.read_text());progress=b/'phase5-current-work-progress-20261026.json';outcome=b/'outcome-policy-deployment.json'
original={p:p.read_bytes() for p in [*updates,receipt_path,progress,outcome]}
for i,(p,data) in enumerate(original.items()):(backup/(str(i)+'-'+p.name)).write_bytes(data)
(backup/'manifest.json').write_text(json.dumps([dict(target=str(p),backup=str(backup/(str(i)+'-'+p.name))) for i,p in enumerate(original)],indent=2))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
try:
 for p,s in updates.items():p.write_text(s)
 env=dict(os.environ,PYTHONPATH=str(b)+':'+str(f))
 tests=subprocess.run(['/usr/bin/python3','-m','unittest','test_penguin_hand_hold','test_penguin_geometry','test_penguin_source_identity','test_pepper_transfer','test_pepper_tasks','-q'],env=env,cwd='/tmp',capture_output=True,text=True)
 (backup/'validation.log').write_text(tests.stdout+tests.stderr)
 if tests.returncode:raise RuntimeError(tests.stderr)
 p=json.loads(progress.read_text())
 for path in updates:
  if path.parent==f:p['latest_expected_hashes'][path.name]=sha(path)
 p['penguin_camera_resolution_revision']=dict(backup=str(backup),source_imgsz_default=960,actual_transfer=str(run),uninterrupted=False)
 progress.write_text(json.dumps(p,indent=2))
 o=json.loads(outcome.read_text());entries={row['name']:row for row in o['installed']};entries['profiled_capture.py']=dict(name='profiled_capture.py',sha256=sha(b/'profiled_capture.py'),source=str(contact/'profiled_capture.py'));o['installed']=list(entries.values());outcome.write_text(json.dumps(o,indent=2))
 for path in [*updates,progress,outcome]:
  rows=[row for row in receipt['files'] if row['target']==str(path)]
  if rows:rows[0]['sha256']=sha(path)
  else:
   idx=list(original).index(path);saved=backup/(str(idx)+'-'+path.name)
   receipt['files'].append(dict(relative='runtime/'+path.name,target=str(path),sha256=sha(path),before_sha256=hashlib.sha256(original[path]).hexdigest(),backup=str(saved)))
 receipt['penguin_revision']=dict(backup=str(backup),actual_transfer=str(run),source_imgsz_default=960,tests=tests.stderr,full_live_transfer_validated=False)
 receipt_path.write_text(json.dumps(receipt,indent=2))
 summary=dict(status='penguin_transfer_verified_repairs_promoted',run=str(run),backup=str(backup),source_imgsz_default=960,uninterrupted_transfer_verified=False,robot_ready=True,robot_state_at_end=result['last_fresh_robot_state'],current_location_is_historical=True,paired_camera_inverse_fixed=True,contact_resume_is_trial_only=True,next='Uninterrupted regression requires a new source object; penguin now on kitchen dining table.')
 (b/'penguin-transfer-checkpoint-20261107.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
except BaseException:
 for p,data in original.items():p.write_bytes(data)
 raise

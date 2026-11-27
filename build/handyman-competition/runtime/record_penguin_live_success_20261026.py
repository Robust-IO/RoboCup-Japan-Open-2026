"""Metadata-only success annotation after verified physical result; no robot IO."""
import hashlib,json,sys
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
checkpoint_path=base/'resident-model-fast-navigation-trial-progress-20261026.json'
cp=json.loads(checkpoint_path.read_text())
if not cp['uninterrupted_transfer_verified'] or not cp['physical_transfer_verified']:raise ValueError('actual_success_required')
roots=(code,repo/'training',repo/'training/automation/apple_transfer_v1')
for root in roots:
 for name,expected in cp['latest_expected_hashes'].items():
  if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:raise ValueError('changed_source:'+str(root/name))
new_catalog=json.loads((base/'penguin-verified-object-catalog-20261026.json').read_text())
old_catalog=json.loads((code/'config/manipulation_objects_v1.json').read_text())
reconstructed=json.loads(json.dumps(new_catalog))
for key in ('status','note'):reconstructed['objects']['toy_penguin'][key]=old_catalog['objects']['toy_penguin'][key]
if reconstructed!=old_catalog:raise ValueError('metadata_patch_changed_control_or_geometry')
old_notes=(code/'OBJECT_TRANSFER.txt').read_bytes();new_notes=(base/'object-transfer-verified-notes-20261026.txt').read_bytes()
if not new_notes.startswith(old_notes):raise ValueError('historical_notes_not_preserved')
sys.path.insert(0,str(code));from manipulation_profiles import object_profile
for name in new_catalog['objects']:object_profile(name,base/'penguin-verified-object-catalog-20261026.json')
for name,source in (('config/manipulation_objects_v1.json','penguin-verified-object-catalog-20261026.json'),('OBJECT_TRANSFER.txt','object-transfer-verified-notes-20261026.txt')):
 data=(base/source).read_bytes();expected=hashlib.sha256(data).hexdigest()
 for root in roots:
  pending=root/(name+'.success-pending');pending.write_bytes(data);pending.replace(root/name)
  if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:raise ValueError('copy_mismatch')
 cp['latest_expected_hashes'][name]=expected
cp['success_annotation_metadata_only']=True
for path in (checkpoint_path,repo/'training'/checkpoint_path.name):
 pending=path.with_suffix('.pending');pending.write_text(json.dumps(cp,indent=2));pending.replace(path)
print(json.dumps(dict(metadata_only=True,all_control_geometry_identity_fields_unchanged=True,three_copies_verified=True,physical_transfer_verified=cp['physical_transfer_verified'])))

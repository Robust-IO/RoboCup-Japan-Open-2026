"""Archive the successful physical experiment; no robot commands or git writes."""
import copy,hashlib,json,tarfile,time,subprocess,sys
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
roots=(code,repo/'training',repo/'training/automation/apple_transfer_v1')
old=json.loads((base/'penguin-table-support-ready-progress-20261025.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for root in roots:
 for name,want in old['latest_expected_hashes'].items():assert sha(root/name)==want,str(root/name)
archive=base/'penguin-staged-success-records-20261025.tar.gz'
assert sha(archive)==sys.argv[1]
validation=base/'penguin-staged-success-records-20261025-validation';validation.mkdir(exist_ok=False)
with tarfile.open(archive) as tar:
 data={}
 for member in tar.getmembers():
  assert member.isfile() and not Path(member.name).is_absolute() and '..' not in Path(member.name).parts
  data[member.name]=tar.extractfile(member).read()
  if member.name.endswith('.py'):compile(data[member.name],member.name,'exec')
new_config=json.loads(data['config/manipulation_objects_v1.json']);prior_config=json.loads((code/'config/manipulation_objects_v1.json').read_text())
a,b=copy.deepcopy(new_config),copy.deepcopy(prior_config)
for key in ('status','note'):
 a['objects']['toy_penguin'].pop(key);b['objects']['toy_penguin'].pop(key)
assert a==b,'only_validation_metadata_may_change'
close=base/'penguin-native-transfer-20261025-10';lift=base/'penguin-current-head-lift-test-20261025-01'
retract=base/'penguin-head-retraction-test-20261025-01';carry=base/'penguin-head-carry-test-20261025-01';place=base/'penguin-head-place-test-20261025-01'
read=lambda p:json.loads(p.read_text())
c,l,r,n,z=[read(p/'result.json') for p in (close,lift,retract,carry,place)]
assert c['close_commands_sent']==1 and all(p['close_commands_sent']==0 for p in (l,r,n,z))
assert z['status']=='penguin_staged_transfer_verified' and z['release_commands_sent']==1
assert r['grasp_success_verified'] and n['carry_destination_verified'] and z['placement_success_verified']
assert all(row['measurement']['current_body_depth_measured'] and row['measurement']['current_head_depth_measured'] for row in z['placed_frames'])
sys.path.insert(0,str(code));from apple_transfer_automation import RobotLock
latest=dict(old['latest_expected_hashes'])
with RobotLock(),tarfile.open(validation/'previous-metadata.tar.gz','x:gz') as backup:
 for i,root in enumerate(roots):
  for name,value in data.items():
   p=root/name;p.parent.mkdir(parents=True,exist_ok=True)
   if p.exists():backup.add(p,arcname=str(i)+'/'+name)
   pending=p.with_name(p.name+'.success-pending');pending.write_bytes(value);pending.replace(p)
   latest[name]=hashlib.sha256(value).hexdigest()
 for root in roots:
  for name,want in latest.items():assert sha(root/name)==want,str(root/name)
 from manipulation_profiles import object_profile
 assert object_profile('toy_penguin')['status']==new_config['objects']['toy_penguin']['status']
 result=dict(old,status='one_staged_penguin_head_grasp_transfer_verified',saved_ns=time.time_ns(),
    latest_expected_hashes=latest,three_deployed_copies_verified=True,tests_passed=706,
    latest_metadata_validation=str(validation),production_control_logic_changed_by_final_archive=False,
    active_object='toy_penguin',source_table='white_side_table#bedroom_1',destination_table='square_low_table#living_room',
    penguin_staged_transfer_verified=True,penguin_single_uninterrupted_automatic_run_verified=False,
    successful_staged_transfers=1,grasp_close_commands_total=1,release_commands_total=1,dropped_during_this_transfer=False,
    stages=dict(close=str(close),lift=str(lift),hand_face_audit=str(base/'penguin-hand-face-lift-follow-audit-20261025.json'),
                retract=str(retract),carry=str(carry),place=str(place)),
    grasp_proof='preclose actual identity/RGBD, preserved contact, measured palm10cm lift and current hand RGB rigidity through retraction/carry',
    carried_pose_source='preclose_RGBD_anchored_kinematic_estimate_with_current_hand_face_RGB_not_new_depth',
    placement_proof='three_new_independent_actual_body_and_head_RGBD_with_current_table_support_and_open_empty_hand',
    final_object_location='square_low_table#living_room',last_verified_robot_state=z['final_state'],
    current_robot_state_not_queried=True,robot_commands_sent=False,checkpoint_write_only=True,
    placed_minimum_outer_clearance_estimate_m=min(p['table_edge_safety']['outer_clearance_lower_estimate_m'] for p in z['placement_verification']['checks']),
    placed_center_spread_m=z['placement_verification']['center_spread_m'],
    model_changed=False,navigation_configuration_changed=False,task_finished_sent=False,
    default_entry_limitation='postclose head depth is finger-occluded; hand-RGB held proof and explicit kinematic-held-pose place path remain experimental and must be integrated',
    next_work=['Integrate hand RGB object motif seed/matching into the default penguin entry; preserve current object identity without fabricating depth.',
               'Reuse genuine preclose geometry as lift baseline after stable contact instead of blocking on redundant postclose full depth.',
               'Use explicit held-palm kinematic pose estimates only when current RGB hold is verified; measure the table from current depth.',
               'Keep independent actual body+head depth and open hand after release as final placement proof.',
               'Then perform reverse reset using config/penguin_living_room_to_bedroom.json and a NEW output directory. Penguin is no longer on the bedroom source table.',
               'Do not blindly rerun the current default full entry: the experimental hand RGB proof is not yet connected.'],
    prior_pepper_checkpoint='ground-pepper-three-rounds-success-progress-20261025.json',
    branch=subprocess.run(['git','branch','--show-current'],cwd=repo,capture_output=True,text=True,check=True).stdout.strip(),
    commit_or_push_performed_this_turn=False)
 name='penguin-first-staged-transfer-success-progress-20261025.json'
 for target in (base/name,repo/'training'/name):
  with target.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps(dict(checkpoint=str(base/name),tests_passed=706,staged_transfer_verified=True,three_code_copies_verified=True,
    default_entry_integration_pending=True,robot_commands_sent=False,tracked_files=len(latest))))

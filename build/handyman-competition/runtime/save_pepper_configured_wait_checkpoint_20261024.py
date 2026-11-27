import json,hashlib,time,sys,tarfile
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');roots=[code,repo/'training',repo/'training/automation/apple_transfer_v1']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
previous=json.loads((base/'pepper-configured-perception-ready-progress-20261024.json').read_text())
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
with RobotLock():
 for root in roots:
  for name,digest in previous['latest_expected_hashes'].items():
   if sha(root/name)!=digest:raise ValueError('deployed_source_changed:'+name)
 doc=base/'OBJECT_TRANSFER.txt'
 backup=base/'pepper-configured-documentation-backup-20261024.tar.gz'
 with tarfile.open(backup,'x:gz') as tar:
  for i,root in enumerate(roots):tar.add(root/'OBJECT_TRANSFER.txt',arcname=str(i)+'/OBJECT_TRANSFER.txt')
 for root in roots:
  pending=root/'OBJECT_TRANSFER.txt.pending-doc-update'
  with pending.open('xb') as f:f.write(doc.read_bytes())
  pending.replace(root/'OBJECT_TRANSFER.txt')
 hashes=dict(previous['latest_expected_hashes']);hashes['OBJECT_TRANSFER.txt']=sha(doc)
 for root in roots:
  for name,digest in hashes.items():
   if sha(root/name)!=digest:raise ValueError('three_copy_mismatch:'+name)
 diagnostic=json.loads((base/'pepper-raw-stream-timing-20261024-03/result.json').read_text())
 result=dict(status='configured_pepper_regression_waiting_unity_time_sync',saved_ns=time.time_ns(),
  branch='feat/handyman-phase5-grasping',boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
  code_ready=True,tests_passed=previous['tests_passed'],latest_expected_hashes=hashes,
  runtime_test_log=previous['latest_validation_directory']+'/tests.log',
  real_rgbd_crop_equivalence='pepper-roi-real-equivalence-20261024.json',
  configured_directions=['bedroom_to_living_room','living_room_to_bedroom'],
  source_identity_processing_separate_from_action_depth=True,
  navigation_speed_route_goal_precision_unchanged=True,
  navigation_joint_age_limit_s=.85,action_joint_age_limit_s=.6,action_image_age_limit_s=.4,
  passed_forward_rounds=0,initial_reverse_reset_verified=False,
  last_regression='pepper-automatic-rounds-20261024-05',
  last_failure='preflight:feedback_wait_expired:stale_joints',
  last_regression_close_commands=0,last_regression_release_commands=0,last_regression_robot_commands_sent=False,
  last_raw_feedback=diagnostic,raw_feedback_is_not_fresh_motion_permission=True,
  expected_current_object_location='square_low_table#living_room',
  last_successful_placement_proof='pepper-native-placement-proof-20261024-12',
  manual_object_reposition_required=False,unity_restart_required=False,
  next_unity_operation='During Play, RosBridgeScripts/SIGVerseRosBridgeTimeSynchronizer: tick Should Synchronize once; preserve scene/object/hand.',
  next_actions=['Read checkpoint and existing05 result; do not count prior staged transfer as uninterrupted round.',
   'After Unity sync, acquire new raw joint/camera ages and actual empty/tucked state. Preserve timestamps; do not change OS clocks.',
   'If feedback restored and pepper still on living table, launch pepper_regression.py --run --rounds 3 --output /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pepper-automatic-rounds-20261024-06.',
   'Let initial automatic reverse reset and three forward rounds with automatic reverse resets run; no manual object moves.',
   'On failure inspect actual held/released counters and current state before another action.'],
  task_finished_sent=False)
 for path in (base/'pepper-configured-regression-waiting-sync-progress-20261024.json',repo/'training/pepper-configured-regression-waiting-sync-progress-20261024.json'):
  with path.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
 print(json.dumps(dict(status=result['status'],tests_passed=result['tests_passed'],passed_forward_rounds=0,three_copies_verified=True)))

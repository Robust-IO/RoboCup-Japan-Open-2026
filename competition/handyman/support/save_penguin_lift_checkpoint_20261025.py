import hashlib,json,time,subprocess
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
ready=json.loads((base/'penguin-source-support-ready-progress-20261025.json').read_text())
for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
 for name,want in ready['latest_expected_hashes'].items():
  assert hashlib.sha256((tree/name).read_bytes()).hexdigest()==want,str(tree/name)
close_root=base/'penguin-native-transfer-20261025-10';lift_root=base/'penguin-current-head-lift-test-20261025-01'
close=json.loads((close_root/'result.json').read_text());lift=json.loads((lift_root/'result.json').read_text())
history=[]
for i in range(1,11):
 p=base/('penguin-native-transfer-20261025-'+str(i).zfill(2))/'result.json'
 r=json.loads(p.read_text());history.append(dict(run=str(p.parent),status=r['status'],stage=r.get('stage'),
     reason=r.get('reason'),close_commands_sent=r['close_commands_sent'],release_commands_sent=r['release_commands_sent']))
result=dict(ready,status='penguin_head_contact_lifted_10cm_visual_hold_present_automation_verification_pending',saved_ns=time.time_ns(),
 branch=subprocess.run(['git','branch','--show-current'],cwd=repo,capture_output=True,text=True,check=True).stdout.strip(),
 active_object='toy_penguin',source='white_side_table#bedroom_1',destination='square_low_table#living_room',
 previous_pepper_success_checkpoint='ground-pepper-three-rounds-success-progress-20261025.json',
 penguin_automatic_transfer_verified=False,penguin_automated_grasp_lift_verified=False,
 penguin_visual_observation='hand camera shows head retained between fingers after 10cm lift; transport not verified',
 penguin_close_run=str(close_root),penguin_lift_test=str(lift_root),penguin_live_attempt_history=history,
 last_confirmed_robot_state=lift['final_state'],last_actual_holding_opening_rad=close['close']['closure_evidence']['observed_opening_rad'],
 current_state_must_be_refreshed_before_more_motion=True,current_hand_should_remain_closed=True,
 tests_passed=705,three_deployed_copies_verified=True,
 unity_clock_sync_completed_by_user=True,post_sync_joint_age_s=.054104268,
 current_blocker='finger occlusion contaminates head depth after closure; current head fitting cannot verify lift',
 pending_plan='verify head/face persistence in calibrated hand RGB across measured palm lift; then verify retraction and carry',
 resume_rules=['READ current actual closed/held state first. Do NOT rerun a fresh full task or close/open the gripper.',
 'One close total, zero release. One diagnostic 10cm lift; arm lift last .42200577m, horizontal, opening .2084151rad.',
 'Preserve source identity and distinguish measured head depth from rigid inferred body. No new object XYZ from Unity.',
 'Latest failed known-radius/noisy-surface candidate was NOT deployed. Local candidate line reverted to the checked 705-test package.',
 'No push/merge/commit requested. All artifacts on D.'],
 robot_commands_sent=False,checkpoint_write_only=True)
name='penguin-head-lift-resume-checkpoint-20261025.json'
for out in (base/name,repo/'training'/name):
 with out.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps(dict(checkpoint=str(base/name),branch=result['branch'],tests_passed=result['tests_passed'],three_deployed_copies_verified=True)))

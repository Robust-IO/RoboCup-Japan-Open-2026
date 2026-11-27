import json,hashlib,sys,time,tarfile
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
from pepper_regression import verify_leg
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
root=base/'pepper-automatic-rounds-20261025-02';probe=read(base/'pepper-stop-readonly-20261025-01/result.json')
ready=read(base/'pepper-source-table-support-ready-progress-20261025.json');roots=[code,repo/'training',repo/'training/automation/apple_transfer_v1']
with RobotLock():
 for tree in roots:
  for name,want in ready['latest_expected_hashes'].items():
   if sha(tree/name)!=want:raise ValueError('unexpected_deployed_edit:'+name)
 report=read(root/'result.json');planned=read(root/'sequence.json')
 if report['passed_rounds']!=1 or len(report['legs'])!=2:raise ValueError('inspect_actual_completed_rounds')
 for i,leg in enumerate(report['legs']):verify_leg(read(Path(leg['output'])/'result.json'),planned[i]['task'])
 stopped=read(root/'legs/02-reset-for-2/result.json')
 if stopped['close_commands_sent']!=0 or stopped['release_commands_sent']!=0:raise ValueError('next_leg_not_empty')
 state=probe.get('current_fresh_state')
 if state is None or state['joints'][5]<.60:raise ValueError('fresh_open_hand_stop_state_required')
 final=read(root/'legs/01-round-1/result.json');proof=read(root/'legs/01-round-1/place/placement-verification.json')
 doc=code/'OBJECT_TRANSFER.txt';note='''\n2026-10-25用户要求本轮结束后停止：\n02组第1轮卧室→客厅完整通过，259.69秒，一次闭爪、一次松爪，三帧实际桌面支撑\n核验及空手收臂全部通过。初始自动回送也完整通过，241.62秒。正向累计1/3轮，\n余下两轮明天继续；不把此前分段恢复算作完整通过。\n协调器自动进入第2轮回送准备后已应用户要求停止，该项闭爪/松爪均为0。空手\n展臂轨迹在停止后自然完成，未再搬动物品；当前为空手、开爪、水平展臂，非收臂\n待命姿态。仅保存只读状态，未再发收臂或搬运指令。胡椒仍在客厅矮桌。\n明天读取ground-pepper-phase5-pause-checkpoint-20261025.json，先核查Unity是否\n重启及当前机器人/物品状态；不要直接重跑旧持物继续模式。保留场景时先处理\n当前空手展臂状态再进行余下两轮；若重启则按新场景原位重新选择搬运方向。\n'''
 with tarfile.open(base/'pepper-pause-documentation-backup-20261025.tar.gz','x:gz') as tar:
  for i,tree in enumerate(roots):tar.add(tree/'OBJECT_TRANSFER.txt',arcname=str(i)+'/OBJECT_TRANSFER.txt')
 payload=doc.read_text()+note
 for tree in roots:
  pending=tree/'OBJECT_TRANSFER.txt.pause-pending'
  with pending.open('x') as f:f.write(payload)
  pending.replace(tree/'OBJECT_TRANSFER.txt')
 hashes=dict(ready['latest_expected_hashes']);hashes['OBJECT_TRANSFER.txt']=sha(doc)
 for tree in roots:
  for name,want in hashes.items():
   if sha(tree/name)!=want:raise ValueError('copy_mismatch:'+name)
 value=dict(status='paused_at_user_request_after_first_completed_forward_round',saved_ns=time.time_ns(),
  branch='feat/handyman-phase5-grasping',boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
  user_requested_stop=True,robot_commands_sent_by_stop_probe=False,no_additional_manipulation_authorized_today=True,
  passed_forward_rounds=1,requested_forward_rounds=3,remaining_forward_rounds=2,
  verified_initial_automatic_reset=report['legs'][0],verified_first_forward_round=report['legs'][1],
  completed_forward_placement_proof=proof,run_root=str(root),
  stopped_next_leg=dict(output=str(root/'legs/02-reset-for-2'),stage=stopped['stage'],close_commands_sent=0,release_commands_sent=0,
    reason='user_stop_after_round_1; queued_next_leg_was_interrupted'),
  current_actual_state_at_save=state,current_state_is_historical_not_future_motion_permission=True,
  robot_holding_object=False,hand_open=True,robot_empty_tucked=probe['empty_tucked'],robot_ready_for_next_task=False,
  current_arm_state='empty_open_horizontal_at_measured_body_height',
  object_at_destination='square_low_table#living_room',object_not_moved_by_interrupted_next_leg=True,
  current_readonly_probe=str(base/'pepper-stop-readonly-20261025-01'),
  tests_passed=675,latest_expected_hashes=hashes,three_deployed_copies_verified=True,
  navigation_speed_route_goal_precision_unchanged=True,task_finished_sent=False,
  fixes_today=['Unity sync restored raw joint median age from .688s to .064s without retimestamping',
    'Optional stopped navigation feedback recovery: confirmed cancel, two new valid scan/TF samples, then unfinished waypoint only; original age limits unchanged.',
    'Current measured source tabletop excludes table-edge background contamination; real failure RGBD replay passed.'],
  prior_split_completion='pepper-stopped-navigation-continuation-20261025-01',
  old_held_context_is_not_current_hold=True,
  resume_instructions=[
   'Respect user stop: today no more robot actions. Tomorrow read this checkpoint first.',
   'Acquire new boot/process/feedback/current RGBD/state. Determine whether Unity restarted; do not command using this historical snapshot.',
   'If same scene: pepper remains on living table, robot empty/open/horizontal. Inspect current state, explicitly fold empty arm only after user resumes; then pepper_regression.py --run --rounds 2 --output NEW_D_DIRECTORY runs two remaining forward rounds with automatic resets.',
   'If Unity restarted: inspect native object location and new empty/tucked base pose. Do not assume pepper is on living table. If pepper is on bedroom white table, run one new normal forward transfer, then one remaining forward round after automatic reverse reset.',
   'Count only full new physical transfer runs; keep completed1 and future2 evidence separately. No repeated grasp/close/release from historical contexts.'
  ])
 for path in (base/'ground-pepper-phase5-pause-checkpoint-20261025.json',repo/'training/ground-pepper-phase5-pause-checkpoint-20261025.json'):
  with path.open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False)
 print(json.dumps(dict(status=value['status'],passed_forward_rounds=1,remaining_forward_rounds=2,
  current_joints=state['joints'],hand_open=True,object_remains_on_living_table=True,three_copies_verified=True)))

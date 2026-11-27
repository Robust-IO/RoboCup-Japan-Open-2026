import json,hashlib,sys,time,tarfile
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(code))
from pepper_regression import verify_leg
from pepper_tasks import load_task
from apple_transfer_automation import RobotLock
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old=read(base/'ground-pepper-phase5-pause-checkpoint-20261025.json')
forward=[Path(old['verified_first_forward_round']['output']),base/'pepper-resume-forward-round-2-20261025-02',base/'pepper-resume-forward-round-3-20261025-01']
last_reset=base/'pepper-lifted-hold-continuation-20261025-01'
ready=read(base/'pepper-lifted-hold-resume-ready-progress-20261025.json')
with RobotLock():
 rows=[]
 for index,path in enumerate(forward,1):
  result=read(path/'result.json');task=result['configured_task'];verify_leg(result,task)
  if task!=load_task():raise ValueError('forward_task_changed')
  proof=read(path/'place/placement-verification.json');navigation=[]
  for name in ('source-navigation.json','destination-navigation.json'):
   nav=read(path/name)
   if nav.get('status')!='arrived' or nav.get('goal_precision_changed') is not False or nav.get('speed_parameters_restored') is not True:raise ValueError('navigation_not_verified')
   navigation.append(dict(name=name,feedback_recoveries=nav.get('feedback_recoveries',0),precision=nav.get('current_precision'),final_pose=nav.get('final_pose')))
  rows.append(dict(round=index,output=str(path),elapsed_s=result['elapsed_s'],close_commands_sent=1,release_commands_sent=1,
   uninterrupted_application_run=True,minimum_table_outer_clearance_m=min(c['table_edge_safety']['outer_clearance_lower_estimate_m'] for c in proof['checks']),
   maximum_bottom_gap_m=max(abs(c['estimated_bottom_gap_m']) for c in proof['checks']),center_spread_m=proof['center_spread_m'],navigation=navigation))
 reset_path=last_reset;reset_result=read(reset_path/'result.json')
 if reset_result.get('physical_transfer_verified') is not True or reset_result.get('robot_ready_for_next_task') is not True or reset_result.get('close_commands_sent')!=0 or reset_result.get('release_commands_sent')!=1 or reset_result.get('continued_from_lifted_hold') is not True:raise ValueError('split_reverse_completion_required')
 roots=[code,repo/'training',repo/'training/automation/apple_transfer_v1']
 for tree in roots:
  for name,want in ready['latest_expected_hashes'].items():
   if sha(tree/name)!=want:raise ValueError('deployed_source_changed:'+name)
 note='''\n2026-10-25恢复后完成胡椒三轮正向验证：\n首轮记录：pepper-automatic-rounds-20261025-02/legs/01-round-1。\n第二轮记录：pepper-resume-forward-round-2-20261025-02（Unity重启初始位姿开始）。\n第三轮记录：pepper-resume-final-round-20261025-01/legs/01-round-1；之前由同组\n00-initial-reset自动回送，未要求手工摆放胡椒。三轮均为完整应用运行，一次闭爪、\n一次松爪、抬升/持物/实际放置/空手收尾通过。三轮跨暂停和Unity重启，不称为同一\n场景中连续三轮，也不据此建立统计成功率。详见成功检查点中的时长和支撑余量。\n\n本次恢复处理了两个环境问题：WSL旧D盘挂载失效（Windows原文件正常），验证\n新挂载可读后恢复/mnt/d路径；Unity重启留下旧ROSBridge会话及等待首条消息的\n临时DDS订阅。按连接活动记录移除旧连接，再在初始空手姿态下发送原位握手，\n五个命令话题恢复单一接收端。未修改ROS安装包、模型、导航速度/路线或精度。\n旧清理工具硬编码本次PID/端口，属于现场恢复记录，禁止直接用于后续其他会话；\n先重新只读核查连接和真实状态。环境处理记录位于rosbridge-old-connection-\nshutdown-verified-20261025.json及rosbridge-pending-subscriber-prime-20261025-02。\n'''
 with tarfile.open(base/'pepper-resume-final-documentation-backup-20261025.tar.gz','x:gz') as tar:
  for i,tree in enumerate(roots):tar.add(tree/'OBJECT_TRANSFER.txt',arcname=str(i)+'/OBJECT_TRANSFER.txt')
 payload=(code/'OBJECT_TRANSFER.txt').read_text()+note
 for tree in roots:
  pending=tree/'OBJECT_TRANSFER.txt.resume-final-pending'
  with pending.open('x') as f:f.write(payload)
  pending.replace(tree/'OBJECT_TRANSFER.txt')
 hashes=dict(ready['latest_expected_hashes']);hashes['OBJECT_TRANSFER.txt']=sha(code/'OBJECT_TRANSFER.txt')
 for tree in roots:
  for name,want in hashes.items():
   if sha(tree/name)!=want:raise ValueError('three_copy_mismatch:'+name)
 final=read(forward[-1]/'result.json')
 value=dict(status='three_complete_ground_pepper_forward_runs_verified_across_resume',saved_ns=time.time_ns(),
  branch='feat/handyman-phase5-grasping',boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),layout='LayoutA',class_name='ground_pepper',
  completed_forward_rounds=3,requested_forward_rounds=3,remaining_forward_rounds=0,new_completed_rounds_after_resume=2,
  rounds=rows,verified_split_final_reset=dict(output=str(reset_path),elapsed_s=reset_result['elapsed_s'],counted_as_uninterrupted_full_transfer=False),
  consecutive_in_one_unity_session=False,all_three_individually_uninterrupted_application_runs=True,manual_object_reposition_required=False,
  statistical_success_rate_established=False,robot_holding_object=False,hand_open=True,robot_empty_tucked=True,robot_ready_for_next_task=True,
  object_at_destination='square_low_table#living_room',last_verified_robot_state=final['last_fresh_robot_state'],last_state_is_historical_not_motion_permission=True,
  task_finished_sent=False,tests_passed=ready['tests_passed'],latest_expected_hashes=hashes,three_deployed_copies_verified=True,navigation_speed_route_goal_precision_unchanged=True,
  pause_checkpoint_superseded=str(base/'ground-pepper-phase5-pause-checkpoint-20261025.json'),
  resume_readonly_probe='pepper-resume-readonly-20261025-01',
  startup_failed_before_robot_motion='pepper-resume-forward-round-2-20261025-01',
  startup_connection_cleanup='rosbridge-old-connection-shutdown-verified-20261025.json',
  startup_neutral_control_handshake='rosbridge-pending-subscriber-prime-20261025-02/result.json',
  fixes_after_resume=['Removed redundant complete-body check immediately after three-frame lift proof','New cap-only real-depth continuation for a previously verified lifted grasp; no inferred body XYZ'],
  next='Verify another LayoutA object already on source furniture using the same route; read actual native position and select its geometry adapter. No manual reposition by default.')
 for path in (base/'ground-pepper-three-rounds-success-progress-20261025.json',repo/'training/ground-pepper-three-rounds-success-progress-20261025.json'):
  with path.open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False)
 print(json.dumps(dict(status=value['status'],completed_rounds=3,remaining=0,round_durations_s=[r['elapsed_s'] for r in rows],final_robot_state=value['last_verified_robot_state'],three_copies_verified=True)),flush=True)

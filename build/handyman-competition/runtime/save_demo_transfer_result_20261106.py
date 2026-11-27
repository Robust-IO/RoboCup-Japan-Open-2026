"""Summarize the observed transfer and recovery without changing old verdicts."""
import json,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
original=b/'cigarette-full-auto-20261106-184003'
mission=json.loads((original/'result.json').read_text())
recovery=b/'cigarette-place-20261101-09'
placed=json.loads((recovery/'result.json').read_text())
if mission['status']!='stopped_requires_state_review' or not placed.get('placement_success_verified'):
    raise RuntimeError('Expected stopped original and independently verified recovery')
if not placed.get('robot_ready_for_next_task') or placed['release_commands_sent']!=1:
    raise RuntimeError('Final task state not ready')
rows=mission['stages']+[dict(stage='place_recovery',run=str(recovery),status=placed['status'],elapsed_s=placed['elapsed_s'])]
active_sum=sum(r['elapsed_s'] for r in rows)
first=json.loads((b/'cigarette-source-20261101-07-launch.json').read_text())['started_ns']
final=placed['final_state']['stamp_ns']
report=dict(status='completed_after_placement_recovery',task=mission['request'],
    original_mission=str(original),original_stopped_result_preserved=True,
    recovery_run=str(recovery),full_uninterrupted_transfer_verified=False,
    physical_transfer_verified=True,placement_success_verified=True,
    robot_ready_for_next_task=True,close_commands_sent=1,release_commands_sent=1,
    task_finished_sent=False,stages=rows,active_phase_sum_s=active_sum,
    observed_wall_from_source_launch_to_final_sensor_s=(final-first)/1e9,
    wall_time_includes_manual_code_repair_wait=True,
    final_state=placed['final_state'],placement_verification=placed['placement_verification'],
    camera_url='http://100.71.59.3:18766',
    active_followup_deployment=str(b/'demo-standoff-recovery-deployment.json'),
    offline_efficiency_package_activated=False,
    tests=dict(relevant_geometry_tests_passed=19),
    next='Apple/cigarette task completed; robot empty tucked at living room table. Do not assume initial pose. A fresh uninterrupted validation still remains.',
    created_ns=time.time_ns())
out=b/'demo-transfer-result-20261106.json';out.write_text(json.dumps(report,ensure_ascii=False,indent=2))
text='香烟搬运实测结果\n\n卧室白桌 → 客厅 square_low_table。\n抓取一次、松爪一次，独立两帧实际深度验证放置成功，机器人已空载收臂。\n\n中途展臂退让复核阻断，保留原失败记录；备份后增加最多五次退让重测，在原持物状态续跑成功。不是一次无中断成功。\n阶段执行时间合计 %.1f 秒；从出发启动到最终传感器记录 %.1f 秒（含现场修复等待）。\n\n当前相机入口：http://100.71.59.3:18766\n当前机器人在客厅目的桌旁，不在初始位置。\n详细记录：%s\n'%(active_sum,(final-first)/1e9,str(out))
(b/'实测结果-20261106.txt').write_text(text,encoding='utf-8-sig')
print(json.dumps(dict(output=str(out),status=report['status'],phase_sum_s=active_sum,wall_s=report['observed_wall_from_source_launch_to_final_sensor_s'],final_pose=placed['final_state']['poses']['map'],placed_captures=[r['capture'] for r in placed['placed_frames']]),ensure_ascii=False))

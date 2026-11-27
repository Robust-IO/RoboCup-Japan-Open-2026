"""Save physical navigation evidence separately from full transfer validation."""
import hashlib,json,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');receipt=json.loads((b/'navigation-efficiency-live-latest.json').read_text());root=Path(receipt['output']);result=json.loads((root/'result.json').read_text())
if result['status']!='round_trip_completed':raise ValueError('round_trip_not_complete:'+result['status'])
legs=[]
for name in ('outbound','return'):
    d=json.loads((root/(name+'.json')).read_text())
    if d['status']!='arrived' or not all(d.get(k) is True for k in ('nav_reset','speed_parameters_restored','tuning_parameters_restored')) or d['goal_precision_changed']:raise ValueError('navigation_completion_or_restore_failed')
    center=[r['assessment']['lateral_m'] for r in d['reached'] if r['kind']=='door_center' and 'assessment' in r]
    legs.append(dict(stage=name,elapsed_s=next(s['elapsed_s'] for s in result['stages'] if s['stage']==name),
        final_pose=d['final_pose'],position_error_m=d['position_error_m'],feedback_recoveries=d['feedback_recoveries'],nav2_retries=len(d['nav2_retries']),
        passed_landmarks=len(d['reached']),door_center_nearby_lateral_max_m=max(center) if center else None,
        startup_ready_s=d['startup_ready_s'],speeds_seen=sorted({tuple(v['requested_caps']) for v in d['speed_contexts']}),
        joint_advisories=sum(e['event']=='joint_feedback_advisory' for e in d['events']),parameters_restored=True))
summary=dict(status='empty_arm_round_trip_validated',root=str(root),created_ns=time.time_ns(),elapsed_s=result['elapsed_s'],legs=legs,
    live_navigation_validated=True,full_live_transfer_validated=False,holding_transport_validated_this_revision=False,
    grasp_commands_sent=False,release_commands_sent=False,task_finished_sent=False,final_pose=result['final_pose'])
(root/'live-summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));(b/'navigation-efficiency-live-summary-latest.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
checkpoint=b/'navigation-efficiency-checkpoint-20261106.json';d=json.loads(checkpoint.read_text());d.update(live_navigation_validated=True,latest_live_navigation=summary,latest_robot_state=dict(pose=result['final_pose'],arm='empty_tucked',gripper_open=True),next='Current robot is back near the living-room origin. Full holding transfer remains to be verified.');checkpoint.write_text(json.dumps(d,ensure_ascii=False,indent=2))
progress=b/'phase5-current-work-progress-20261026.json';d=json.loads(progress.read_text());d['navigation_efficiency_v1'].update(live_navigation_validated=True,live_evidence=str(root/'live-summary.json'));progress.write_text(json.dumps(d,ensure_ascii=False,indent=2))
dep_path=b/'navigation-efficiency-deployment-latest.json';dep=json.loads(dep_path.read_text());dep.update(live_navigation_validated=True,latest_live_navigation=str(root/'live-summary.json'))
for row in dep['files']:
    if row['target']==str(progress):row['sha256']=hashlib.sha256(progress.read_bytes()).hexdigest()
dep_path.write_text(json.dumps(dep,indent=2))
text='导航效率 V1 空载实测结果\n\n客厅→卧室→客厅，已完成。\n'
for leg in legs:text+=f"{leg['stage']}: {leg['elapsed_s']:.1f}秒，最终位置误差{leg['position_error_m']*100:.1f}厘米，反馈恢复{leg['feedback_recoveries']}次，Nav2重试{leg['nav2_retries']}次。\n"
text+=f"总用时{result['elapsed_s']:.1f}秒。参数均已恢复，机器人空载收臂回到客厅出发位置附近。\n本轮未抓放物品，不能视为新版完整持物搬运通过。\n详细证据：{root}\n"
(b/'导航效率V1-实测结果-20261106.txt').write_text(text,encoding='utf-8-sig');print(json.dumps(summary,ensure_ascii=False))

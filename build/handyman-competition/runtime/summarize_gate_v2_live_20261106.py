"""Read completed mission evidence; never publishes robot commands or changes verdicts."""
import hashlib,json,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');launch=json.loads((b/'cigarette-full-auto-latest.json').read_text());root=Path(launch['output'])
if root.name!='cigarette-full-auto-20261106-200422':raise ValueError('expected_this_live_regression_only')
r=json.loads((root/'result.json').read_text())
if r.get('status')!='completed' or not r.get('physical_transfer_verified'):raise ValueError('mission_not_yet_verified_complete')
if [s['stage'] for s in r['stages']]!=['source','grasp','carry','place']:raise ValueError('four_stages_not_observed')
stages={k:json.loads((Path(r[k])/'result.json').read_text()) for k in ('source','grasp','carry','place')}
g,c,p=stages['grasp'],stages['carry'],stages['place']
if not g['grasp_success_verified'] or not c['carry_destination_verified'] or not p['placement_success_verified']:raise ValueError('physical_stage_not_verified')
if g['close_commands_sent']!=1 or g['release_commands_sent']!=0 or p['release_commands_sent']!=1:raise ValueError('unexpected_grasp_or_release_repetition')
deployment=json.loads((b/'gate-revision-deployment-latest.json').read_text())
for row in deployment['files']:
    if hashlib.sha256(Path(row['target']).read_bytes()).hexdigest()!=row['sha256']:raise ValueError('code_or_deployment_changed_during_trial')
navigation={k:json.loads((Path(r[k])/('source-navigation.json' if k=='source' else 'destination-navigation.json')).read_text()) for k in ('source','carry')}
events={}
for k in ('source','grasp','carry','place'):
    path=Path(r[k])/'journal.jsonl';events[k]=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
report=dict(status='full_automatic_transfer_verified',mission=str(root),task=r['request'],elapsed_s=r['elapsed_s'],stages=r['stages'],initial_state=r['initial_state'],final_state=p['final_state'],
    full_uninterrupted_orchestration_verified=True,stage_relaunches=0,operator_identity_fallback_used=r['operator_identity_fallback_used'],
    physical_transfer_verified=True,close_commands_sent=1,release_commands_sent=1,robot_ready_for_next_task=p['robot_ready_for_next_task'],
    source_detection_used=g.get('source_model_detection_used'),source_geometry_reobservations=sum(e.get('event')=='source_geometry_reobserve' for e in events['grasp']),
    preextension_standoff_corrections=p.get('preextension_standoff_corrections',0),
    navigation_feedback_recoveries={k:v.get('feedback_recoveries',0) for k,v in navigation.items()},
    navigation_elapsed_s={k:next((e2['time_ns']-e1['time_ns'])/1e9 for e1 in events[k] for e2 in events[k] if e1.get('event')=='command_intent' and e2.get('event')=='command_confirmed' and e1.get('sequence')==e2.get('sequence') and e1.get('stage') in ('navigate_source','navigate_destination')) for k in ('source','carry')},
    placement_verification=p['placement_verification'],placed_frames=[v['capture'] for v in p['placed_frames']],
    task_finished_sent=r['task_finished_sent'],code_changed_during_trial=False,
    interpretation='One orchestration completed all four stages with no stage relaunch or code change. The placement-only adapter retains its conservative single_uninterrupted_run=false field because it does not establish full mission scope.',
    comparison_limit='Start was the previous living-room table pose, not the original Unity entry pose; previous trials are not controlled timing comparisons.',
    deployment=deployment['candidate'],backup=deployment['backup'],created_ns=time.time_ns())
(root/'live-regression-summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));(b/'gate-v2-live-regression-latest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
text='门槛调整V2 实测结果\n\n任务：卧室白桌香烟 → 客厅矮方桌。\n本轮从此前客厅目的桌旁出发，四阶段由同一个调度器连续完成，未重启阶段、未修改运行代码。\n总耗时：%.2f秒。\n'%r['elapsed_s']
text+='\n'.join('%s：%.2f秒'%(s['stage'],s['elapsed_s']) for s in r['stages'])
text+='\n\n闭爪1次，松爪1次；独立实际深度验证抓起和落桌，机器人已空载收臂。\n导航反馈恢复次数：%s。\n展臂退让次数：%s。\n\n这是单轮成功，尚不代表多轮可靠性或其他物品验证完成。\n完整记录：%s\n'%(report['navigation_feedback_recoveries'],report['preextension_standoff_corrections'],str(root))
(b/'门槛调整V2-实测结果-20261106.txt').write_text(text,encoding='utf-8-sig')
checkpoint=json.loads((b/'phase5-gate-revision-checkpoint-20261106.json').read_text());checkpoint.update(full_live_transfer_validated=True,latest_live_regression=str(root/'live-regression-summary.json'),latest_robot_state=p['final_state']);(b/'phase5-gate-revision-checkpoint-20261106.json').write_text(json.dumps(checkpoint,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('initial_state','final_state','placement_verification','interpretation')},ensure_ascii=False))

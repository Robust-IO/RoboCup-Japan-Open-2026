"""Read existing files/journals only; never imports ROS or controls the robot."""
import collections,hashlib,json
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');checkpoint=json.loads((b/'phase5-table-localization-checkpoint-20261101.json').read_text());runs=['cigarette-source-20261101-05',checkpoint['grasp'],checkpoint['carry'],Path(checkpoint['place']).name]
out=dict(scope='Nov1 cigarette workflow; repaired runs, not one uninterrupted benchmark',robot_commands_sent=False,files={},runs=[],localization=[])
for name in ('outcome_transfer_runtime.py','canned_grasp_runtime.py','flat_box_grasp.py','flat_box_place.py','flat_box_carry.py','table_localization.py','table_localization_client.py','table_localization_service.py','stop_observe_act.py','outcome_finite_adjustment.py'):
 p=b/name
 if not p.exists():p=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')/name
 out['files'][name]=dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
for name in runs:
 root=b/name;r=json.loads((root/'result.json').read_text());j=root/'journal.jsonl';events=[json.loads(s) for s in j.read_text().splitlines() if s.strip()] if j.exists() else []
 commands=[]
 for i,e in enumerate(events):
  if e.get('event') in ('horizontal_joint_sent','finite_can_approach_sent','task_head_goal_sent','can_closure_sent'):
   next_e=next((x for x in events[i+1:] if x.get('event') in ('horizontal_joint_sent','finite_can_approach_sent','task_head_goal_sent','can_closure_sent','actual_box_depth','command_intent','command_confirmed','head_pose_verified_from_own_joint_feedback')),None)
   command=dict(event=e['event'],label=e.get('label'),duration_s=e.get('duration_s'),time_ns=e['time_ns'])
   if next_e:command.update(next_event=next_e['event'],to_next_event_s=(next_e['time_ns']-e['time_ns'])/1e9)
   commands.append(command)
 row=dict(run=name,status=r['status'],elapsed_s=r.get('elapsed_s'),reason=r.get('reason'),event_counts=dict(collections.Counter(e.get('event') for e in events)),commands=commands)
 row['journal_span_s']=(events[-1]['time_ns']-events[0]['time_ns'])/1e9 if len(events)>1 else None
 row['child_commands']=[dict(arguments=e.get('command'),elapsed_to_confirmation_s=next(((x['time_ns']-e['time_ns'])/1e9 for x in events if x.get('event')=='command_confirmed' and x.get('sequence')==e.get('sequence')),None)) for e in events if e.get('event')=='command_intent']
 row['localization']=[dict(name=p.name,correction_applied=(v:=json.loads(p.read_text())).get('correction_applied'),scan_span_s=(v['scans'][-1]['stamp_ns']-v['scans'][0]['stamp_ns'])/1e9) for p in root.glob('*-localization.json') if 'scans' in json.loads(p.read_text())]
 for p in root.glob('*approach*.json'):
  try:
   val=json.loads(p.read_text())
   if 'duration_s' in val:row.setdefault('finite_adjustments',[]).append(dict(name=p.name,duration_s=val['duration_s'],status=val.get('status'),delta=val.get('delta')))
  except (ValueError,KeyError):pass
 out['runs'].append(row)
service=Path(checkpoint['service']['output'])
for p in sorted(service.glob('check-*.json')):
 if p.name.endswith('-scans.json'):continue
 v=json.loads(p.read_text());scans=v.get('scans',[])
 if scans:
  out['localization'].append(dict(file=p.name,status=v['status'],correction_applied=v.get('correction_applied'),
   first_to_last_scan_s=(scans[-1]['stamp_ns']-scans[0]['stamp_ns'])/1e9,
   last_scan_to_report_file_s=(p.stat().st_mtime_ns-scans[-1]['stamp_ns'])/1e9,
   min_current_inlier=min(x['current_inlier_fraction'] for x in scans),min_matched_inlier=min(x['matched_inlier_fraction'] for x in scans)))
out['configs']={name:json.loads((b/name).read_text()) for name in ('outcome_transfer_policy.json','table_localization.json')}
(b/'defensive-boundary-audit-20261106.json').write_text(json.dumps(out,ensure_ascii=False,indent=2));print(json.dumps(out,ensure_ascii=False))

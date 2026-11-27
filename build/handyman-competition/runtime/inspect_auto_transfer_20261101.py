"""Read-only compact status for the latest full automatic cigarette transfer."""
import json
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');receipt=json.loads((b/'cigarette-full-auto-latest.json').read_text());root=Path(receipt['output'])
def read(p):
    try:return json.loads(p.read_text())
    except (FileNotFoundError,json.JSONDecodeError):return None
report=read(root/'result.json');print(json.dumps(dict(receipt=receipt,result=report),ensure_ascii=False))
if report:
    active=report.get('active_stage');path=Path(report.get(active,str(root)))
    status=read(path/'result.json')
    if status:
        keys=('status','stage','reason','elapsed_s','last_robot_state','initial_base_map_pose','grasp_success_verified','placement_success_verified','carry_destination_verified','close_commands_sent','release_commands_sent','approach_commands_sent','last_target','stages')
        print(json.dumps(dict(active_output=str(path),stage_result={k:status[k] for k in keys if k in status}),ensure_ascii=False))
    for p in (path/'journal.jsonl',path/'source-navigation/result.json',path/'destination-navigation/result.json'):
        if p.exists():
            if p.suffix=='.jsonl':print(str(p)+'\n'+'\n'.join(p.read_text().splitlines()[-2:])[-3500:])
            else:print(str(p)+'\n'+p.read_text()[-1800:])
    log=b/(path.name+'-console.log')
    if log.exists():print('console_tail\n'+log.read_text(errors='replace')[-1800:])

"""Read existing task evidence only. Never retries, resumes or publishes control."""
import json
from pathlib import Path
import subprocess

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'automatic-apple-r16b-20261022-01'
control=base/'apple-r16b-full-live-control-20261022-01'
read=lambda p:json.loads(p.read_text())
result={}
if (root/'status.json').exists():result['status']=read(root/'status.json')
if (root/'result.json').exists():result['result']=read(root/'result.json')
if (root/'journal.jsonl').exists():
    lines=(root/'journal.jsonl').read_text().splitlines();result['last_events']=[json.loads(x) for x in lines[-4:]]
if (control/'automation.log').exists():result['automation_tail']=(control/'automation.log').read_text(errors='replace')[-2500:]
logs=sorted(root.glob('command-*.log'))
if logs:result['current_command_log']=str(logs[-1]);result['command_tail']=logs[-1].read_text(errors='replace')[-3500:]
for folder in ('grasp','grasp/close-lift','place-release','place-raise','place-retract'):
    p=root/folder/'result.json'
    if p.exists():
        value=read(p)
        result[folder]={k:value[k] for k in ('status','stage','reason','error','grasp_success_verified','effective_grasp_success_verified','placement_verified','final_joints') if k in value}
print(json.dumps(result,ensure_ascii=False),flush=True)

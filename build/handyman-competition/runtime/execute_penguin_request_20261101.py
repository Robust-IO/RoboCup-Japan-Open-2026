"""Run the user's bedroom white-table penguin delivery in a new detached run."""
import json,os,subprocess,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
name='penguin-bedroom-white-table-to-living-low-table-20261101-01';root=b/name
if any((b/(name+s)).exists() for s in ('','-console.log','-launch.json')):raise ValueError('existing_delivery_must_be_reviewed')
command=['bash',str(code/'run_object_transfer.sh'),'--object','toy_penguin','--run',
    '--task-config',str(code/'config/penguin_bedroom_to_living_room.json'),
    '--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision',
    '--navigation-profile','carry_fast_v1','--output',str(root)]
with (b/(name+'-console.log')).open('xb') as log:
    child=subprocess.Popen(command,cwd=code,env=dict(os.environ,HANDYMAN_RESIDENT_MODEL='1'),
        stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),request='卧室白色桌子上的企鹅→客厅矮方桌',command=command,output=str(root),
    adaptation='verified_penguin_head_grasp_with_hand_RGB_hold_and_actual_placed_RGBD_verification',
    thin_box_strategy_used=False,protocol_task_finished_sent=False)
with (b/(name+'-launch.json')).open('x') as stream:json.dump(receipt,stream,ensure_ascii=False,indent=2)
print(json.dumps(receipt,ensure_ascii=False))

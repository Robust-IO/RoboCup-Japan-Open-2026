"""Launch placement once this task's verified carry completes; never replay it."""
import json,subprocess,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');carry=base/'cigarette-carry-20261101-03'
receipt=base/'cigarette-place-20261101-05-launch.json'
deadline=time.monotonic()+40
while True:
    if receipt.exists():
        print(json.dumps(dict(status='placement_already_launched',receipt=json.loads(receipt.read_text()))));break
    report=json.loads((carry/'result.json').read_text())
    if report.get('carry_destination_verified') is True and report.get('release_commands_sent')==0:
        held=json.loads((carry/'held-context.json').read_text())
        if Path(held['source_run']).name!='cigarette-contact-20261101-02':raise ValueError('current_task_grasp_required')
        subprocess.run(['/usr/bin/python3',str(base/'launch_flat_box_stage_20261101.py'),'place','--attempt','5',
            '--held-run',str(carry),'--gripper-geometry',str(base/'hsr-gripper-colliders-20261101-171937.json')],check=True)
        break
    if report.get('status')!='starting' or time.monotonic()>=deadline:
        print(json.dumps(dict(status='not_launched',carry_status=report.get('status'),reason=report.get('reason'))));break
    time.sleep(2)

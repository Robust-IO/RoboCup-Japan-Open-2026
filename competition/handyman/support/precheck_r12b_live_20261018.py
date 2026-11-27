"""Read-only current-scene/empty-state evidence before ONE new r12b trial."""
import json
import os
from pathlib import Path
import subprocess
import sys

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,check_inputs,write_json
if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise ValueError('domain71_localhost_required')
base=Path('/__HANDYMAN_PACKAGE__/runtime');out=base/'apple-r12b-live-precheck-20261018-01'
cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
if cfg['policy'].get('near_grasp_geometry_handoff') is not True or cfg['policy']['near_grasp_geometry_gap_m']!=.15:
    raise ValueError('r12b_handoff_not_installed')
with RobotLock():
    out.mkdir(exist_ok=False);write_json(out/'input-files.json',check_inputs(cfg))
    processes=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:args=[v.decode(errors='replace') for v in (p/'cmdline').read_bytes().split(b'\0') if v]
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
        if any('handyman_initial_pose_localizer' in v or Path(v).name in
               ('rviz2','apple_transfer_automation.py','run_bedroom_apple_regrasp.py',
                'run_apple_close_lift_trial.py','pregrasp_raised_trial.py','reviewed_carry_adjustment.py') for v in args):
            processes.append(dict(pid=int(p.name),args=args))
    write_json(out/'processes.json',dict(processes=processes,robot_commands_sent=False))
    commands={
        'ready':[sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(out/'ready.json')],
        'cameras':[sys.executable,str(code/'capture_grasp_scene.py'),str(out/'cameras'),
            '--hand-geometry','--map-geometry','--tracking-epoch',out.name,'--duration','7','--interval','1'],
        'scans':[sys.executable,str(base/'source-table-boundary-audit-20261017-01/capture_table_localization_audit.py'),
            '--output',str(out/'scans.json')]
    }
    children={};logs={}
    for name,command in commands.items():
        logs[name]=(out/(name+'.log')).open('x')
        children[name]=subprocess.Popen(command,stdout=logs[name],stderr=subprocess.STDOUT,start_new_session=True)
    codes={name:child.wait(timeout=40) for name,child in children.items()}
    for log in logs.values():log.close()
    captures=sorted((out/'cameras').glob('*/snapshot.json'))
    report=dict(status='current_readonly_evidence_collected',exit_codes=codes,
        last_capture=str(captures[-1].parent) if captures else None,robot_commands_sent=False,
        readiness=json.loads((out/'ready.json').read_text()) if (out/'ready.json').exists() else None,
        processes=processes,requires_scene_localization_review=True)
    write_json(out/'result.json',report);print(json.dumps(report),flush=True)
    if any(codes.values()):raise SystemExit(1)

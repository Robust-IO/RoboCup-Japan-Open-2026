"""Reuse original LayoutA navigation/head view ONLY, then record raw RGBD.

Does not call the apple observation, prepare, grasp, lift or placement stages.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(CODE))
from apple_transfer_auto_backend import LiveBackend
from apple_transfer_automation import RobotLock,check_inputs,write_json

ROOT=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/canned-source-observation-20261023-01')
ROOT.mkdir(exist_ok=False)
cfg=json.loads((CODE/'config/apple_transfer_auto_v1.json').read_text())
report=dict(status='starting',class_name='canned_juice',base_and_head_only=True,
    arm_commands_sent=False,gripper_commands_sent=False,physical_transfer_verified=False,
    navigation_tolerances_changed=False,automatic_retry=False)
with RobotLock(),(ROOT/'journal.jsonl').open('x') as stream:
    def record(event):
        value=dict(time_ns=time.time_ns(),**event)
        stream.write(json.dumps(value)+'\n');stream.flush()
        if value['event'] in ('stage_start','stage_confirmed'):print(json.dumps(value),flush=True)
    backend=LiveBackend(cfg)
    backend.bind(ROOT,'canned-observe-20261023-01',record)
    try:
        write_json(ROOT/'input-audit.json',check_inputs(cfg))
        backend.stage='preflight'
        ready=backend.command([CODE/'inspect_transfer_ready.py','--output',ROOT/'preflight.json','--efficient-transfer'],
            ROOT/'preflight.json','ready',True,25)
        backend.start_room=backend.room_at(ready['base_map_pose'])
        if backend.start_room not in ('living_room','bedroom'):raise ValueError('outside_existing_source_route')
        backend.initial_opening=.65 if ready['initial_hand_open'] else 0.
        for stage in ('navigate_source','head_view'):
            report['stage']=stage;write_json(ROOT/'status.json',report)
            record(dict(event='stage_start',stage=stage))
            result=backend.perform(stage)
            if result.get('verified') is not True:raise ValueError('stage_not_verified:'+stage)
            record(dict(event='stage_confirmed',stage=stage,result=result))
        backend.stage='record_canned_rgbd'
        path=ROOT/'cameras'
        backend.command([CODE/'capture_grasp_scene.py',path,'--hand-geometry','--map-geometry',
            '--tracking-epoch','canned-source-20261023-01','--duration','10','--interval','.6',
            '--stop-after-frames','3','--min-frame-span','1'],path/'manifest.json',timeout=30)
        frames=sorted(path.glob('*/snapshot.json'))
        if len(frames)!=3:raise ValueError('three_new_captures_required')
        report.update(status='source_observation_recorded',captures=[str(p.parent) for p in frames],
            stage='observe_only',identity_confirmed=False,grasp_started=False)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',reason=str(exc) or 'interrupted')
    finally:
        backend.close()
        write_json(ROOT/'result.json',report)
        print(json.dumps(report),flush=True)
if report['status']!='source_observation_recorded':raise SystemExit(1)

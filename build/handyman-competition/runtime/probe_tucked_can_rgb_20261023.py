"""Read-only current head RGB classification in the already tucked held pose."""
import json
import os
from pathlib import Path
import sys
import time

CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json,check_inputs
from canned_grasp_runtime import CanRuntime
from canned_carry_geometry import current_images_and_feedback,tucked_holding
from apple_worker_client import Worker
import rclpy

BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=BASE/'tucked-can-rgb-probe-20261023-01';root.mkdir(exist_ok=False)
cfg=json.loads((CODE/'config/apple_transfer_auto_v1.json').read_text())
reference=json.loads((BASE/'canned-grasp-lift-live-progress-20261023.json').read_text())
os.environ['YOLO_CONFIG_DIR']=str(BASE/'generic-object-yolo-settings')
report=dict(robot_commands_sent=False,classification_only=True,measured_3d_can_position=False)
with RobotLock():
    write_json(root/'input-audit.json',check_inputs(cfg));rclpy.init()
    runtime=CanRuntime(root,root.name,lambda e:None);worker=None;rows=[]
    try:
        worker=Worker(cfg['model_python'],cfg['registry'],root/'worker.log',imgsz=960)
        print(json.dumps(dict(stage='current_model_ready',model=worker.ready)),flush=True)
        for i in range(3):
            sample=runtime.sample();s=sample['snapshot'];path=runtime.save(sample,f'frame-{i:02d}')
            prediction=worker.infer(path,pump=lambda:runtime.pump(0.))
            state=runtime.state();timing=current_images_and_feedback(s,state,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
            if not tucked_holding(state['joints'],reference['held_opening_rad']):raise ValueError('not_current_tucked_holding_pose')
            value=dict(capture=str(path),detections=prediction.get('detections',[]),timing=timing,
                model_sha256=worker.ready['model_sha256'],robot_state=state,robot_commands_sent=False)
            write_json(path/'classification.json',value);rows.append(value)
        report.update(status='fresh_tucked_rgb_classified',rows=rows)
    except Exception as exc:report.update(status='probe_incomplete',reason=str(exc),rows=rows)
    finally:
        if worker:worker.close()
        runtime.shutdown();rclpy.shutdown();write_json(root/'result.json',report)
        print(json.dumps(report),flush=True)

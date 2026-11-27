"""Serialize the no-motion REAL source pipeline validation, no replay/control."""
import json
import os
from pathlib import Path
import subprocess
import sys

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
base=Path('/__HANDYMAN_PACKAGE__/runtime');out=base/'apple-capture-budget-r13-source-readonly-20261018-01'
deployed=json.loads((base/'apple-capture-budget-r13-20261018-deployment.json').read_text())
if deployed['status']!='deployed_tests_passed':raise ValueError('tested_deployment_required')
if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('domain71_required')
with RobotLock():
    command=[sys.executable,str(code/'verify_source_timing_readonly.py'),'--output',str(out),'--samples','5']
    process=subprocess.run(command,check=False)
    report=json.loads((out/'result.json').read_text())
    okay=process.returncode==0 and report.get('all_grasp_plans_eligible') is True and report.get('samples',0)>=5
    value=dict(status='live_source_pairing_identity_and_plans_verified' if okay else 'live_source_validation_not_passed',
        evidence=str(out/'result.json'),samples=report.get('samples'),reason=report.get('reason'),
        plan_rejections=report.get('grasp_plan_rejections'),robot_commands_sent=False,
        new_full_trial_permitted_by_this_readonly_evidence=okay,
        warning='Read-only validation, NOT a grasp/placement result or reusable future live lease')
    with (out/'summary.json').open('x') as stream:json.dump(value,stream,indent=2)
    print(json.dumps(value),flush=True)
    raise SystemExit(0 if okay else 1)

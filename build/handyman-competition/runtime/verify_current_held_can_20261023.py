"""Read-only CURRENT held-can check, no close/lift/release or other publisher."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np

parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,required=True);args=parser.parse_args()
sys.path.insert(0,str(args.candidate))
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime
from canned_grasp_geometry import fresh,lift_follow,measure_near,predicted_before_contact,transform_point
from run_apple_close_lift_trial import lift_allowed
import scipy.sparse.csgraph
import rclpy

BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');source=BASE/'canned-close-lift-continuation-20261023-01'
root=BASE/'canned-current-held-verification-20261023-01';root.mkdir(exist_ok=False)
trial=json.loads((source/'result.json').read_text())
if not lift_allowed(trial.get('close',{})) or trial.get('lift',{}).get('status')!='passed':
    raise ValueError('actual_preceding_contact_and_lift_required')
reference=json.loads((Path(trial['predecessor'])/'locked-can-reference.json').read_text())
before=Path(trial['close']['before_capture']);bs=json.loads((before/'snapshot.json').read_text())
bg=measure_near(np.load(before/'depth_m.npy',allow_pickle=False),bs,
    predicted_before_contact(reference,bs,reference['task_id']),reference['radius_m'],reference['height_m'])
c0=transform_point(bs,bg['center_m']);p0=transform_point(bs,bs['palm_base_at_depth'])
opening=trial['close']['closure_evidence']['observed_opening_rad']
rclpy.init();runtime=CanRuntime(root,reference['epoch'],lambda v:print(json.dumps(v),flush=True));accepted=[];rejected=[]
report=dict(status='checking',robot_commands_sent=False,grasp_lift_verified=False,placement_verified=False)
try:
    with RobotLock():
        end=time.monotonic()+8
        while len(accepted)<3 and time.monotonic()<end:
            sample=runtime.sample();snap=sample['snapshot']
            try:
                p1=transform_point(snap,snap['palm_base_at_depth']);expected=transform_point(snap,c0+p1-p0,inverse=True)
                measured=measure_near(sample['depth'],snap,expected,reference['radius_m'],reference['height_m'])
                proof=lift_follow(bg,bs,measured,snap,opening)
                fresh(snap,runtime.node.get_clock().now().nanoseconds,reference['epoch'])
                if accepted and proof['stamp_ns']-accepted[-1]['stamp_ns']<200_000_000:continue
                path=runtime.save(sample,f'held-{len(accepted):02d}',measured)
                accepted.append(dict(proof,measurement=measured,capture=str(path)))
            except ValueError as exc:
                path=runtime.save(sample,f'rejected-{len(rejected):02d}')
                rejected.append(dict(reason=str(exc),capture=str(path)))
        relative=np.array([row['center_minus_palm_odom_m'] for row in accepted])
        if len(accepted)!=3 or np.max(np.linalg.norm(relative-relative.mean(0),axis=1))>.02:
            raise ValueError('new_current_held_can_not_verified')
        report.update(status='current_can_held_after_actual_lift_verified',grasp_lift_verified=True,
            held_opening_rad=opening,rows=accepted,rejected=rejected,last_fresh_robot_state=runtime.state(),
            source_closed_once=str(source),new_actions_performed=0,
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
except Exception as exc:report.update(status='verification_inconclusive',reason=str(exc),rows=accepted,rejected=rejected)
finally:
    runtime.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if not report['grasp_lift_verified']:raise SystemExit(1)

"""Validate isolated height adapter and replay the actual kitchen observation."""
import json,os,subprocess,sys
from pathlib import Path
import numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');receipt=json.loads((b/'pepper-dynamic-height-adapter-latest.json').read_text());adapter=Path(receipt['adapter']);root=Path(receipt['predecessor']);sys.path[:0]=[str(adapter),str(b)]
env=dict(os.environ,PYTHONPATH=str(adapter)+':'+str(b))
tests=subprocess.run([sys.executable,'-m','unittest','test_dynamic_table_height','test_pepper_tasks','test_pepper_geometry','test_pepper_transfer','test_motion_speed','-q'],env=env,capture_output=True,text=True)
(adapter/'validation-tests.log').write_text(tests.stdout+tests.stderr)
if tests.returncode:print((tests.stdout+tests.stderr)[-5000:]);raise SystemExit(tests.returncode)
from can_destination_surface import measure
from pepper_place import make_plan,extension_clearance
from apple_transfer_auto_plan import map_point,map_pose
from pregrasp_geometry import expected_horizontal_center
from table_approach_reachability import base_path
from hand_camera_ray import rotation
from dynamic_table_height import lift_targets
capture=root/'place/captures/table-before-extension';s=json.loads((capture/'snapshot.json').read_text());depth=np.load(capture/'depth_m.npy')
held=json.loads((root/'held-context.json').read_text());cfg=json.loads((root/'config.snapshot.json').read_text());dest=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==cfg['destination_id'])
plane=measure(depth,s,dest);plan=make_plan(depth,s,dest,plane,held,cfg,'recorded-kitchen-replay');pred=map_point(s,expected_horizontal_center(plan['palm_height_m'],held['body_minus_palm_horizontal_m']));clearance=extension_clearance(pred,s,plane,plan)
delta=rotation(s['tracking_transforms']['map_from_base_at_depth']['quaternion']).T@np.r_[np.array(plan['release_xy_map_m'])-pred[:2],0.]
reach=base_path(map_pose(s),[*delta[:2],0.],dest)
report=dict(status='validated_recorded_geometry',test_output=(tests.stdout+tests.stderr)[-800:],plan=plan,dynamic_heights=lift_targets(plan['palm_height_m']),extension_clearance=clearance,base_reachability=reach,robot_commands_sent=False)
(adapter/'recorded-kitchen-replay.json').write_text(json.dumps(report,indent=2));receipt['offline_validation']=str(adapter/'recorded-kitchen-replay.json');(b/'pepper-dynamic-height-adapter-latest.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='plan'}))

import hashlib,json,sys,unittest
from pathlib import Path
import numpy as np
from PIL import Image
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
sys.path[:0]=[str(base),str(code)]
from horizontal_can_geometry import estimate,body_appearance,measure_near
suite=unittest.defaultTestLoader.loadTestsFromNames(['test_horizontal_can_geometry','test_horizontal_can_grasp_plan'])
result=unittest.TextTestRunner(verbosity=1).run(suite)
if not result.wasSuccessful():raise SystemExit(1)
prior=json.loads((base/'phase5-horizontal-can-progress-20261101.json').read_text())
repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
    for name,sha in prior['latest_expected_hashes'].items():
        if hashlib.sha256((tree/name).read_bytes()).hexdigest()!=sha:raise ValueError('frozen_code_changed:'+str(tree/name))
rows=[]
for folder in sorted((base/'horizontal-can-source-20261101-01/captures').iterdir()):
    rgb=np.asarray(Image.open(folder/'rgb.png').convert('RGB'));depth=np.load(folder/'depth_m.npy',allow_pickle=False)
    s=json.loads((folder/'snapshot.json').read_text());d=json.loads((folder/'detections.json').read_text())['detections']
    target=next(x for x in d if x['name']=='canned_juice');m=estimate(rgb,depth,s,target)
    appearance=body_appearance(rgb,depth,s,m)
    measured=measure_near(rgb,depth,s,m['center_base_m'],m['axis_base'],m,appearance)
    error=float(np.linalg.norm(np.asarray(m['center_base_m'])-measured['center_m']))
    if error>.006:raise ValueError('source_and_near_fit_disagree:'+str(error))
    rows.append(dict(capture=str(folder),center_difference_m=error,appearance=appearance,measurement=measured))
report=dict(tests_passed=result.testsRun,actual_rgbd_replays=rows,existing_three_code_trees_unchanged=True,
            new_trial_files={name:hashlib.sha256((base/name).read_bytes()).hexdigest() for name in
                             ['horizontal_can_geometry.py','horizontal_can_grasp_plan.py','horizontal_can_trial.py']},
            robot_commands_sent=False)
with (base/'horizontal-can-validation-20261101-02.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report))

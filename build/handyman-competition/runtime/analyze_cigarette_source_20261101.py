import json,sys,unittest
from pathlib import Path
import numpy as np
from PIL import Image
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
sys.path[:0]=[str(base),str(code)]
from flat_box_geometry import estimate,appearance_from_source,measure_near
tests=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromName('test_flat_box_geometry'))
if not tests.wasSuccessful():raise SystemExit(1)
root=base/'cigarette-source-20261101-01';rows=[]
for capture in sorted((root/'captures').iterdir()):
    s=json.loads((capture/'snapshot.json').read_text());rgb=np.asarray(Image.open(capture/'rgb.png').convert('RGB'))
    depth=np.load(capture/'depth_m.npy',allow_pickle=False)
    candidates=[d for d in json.loads((capture/'detections.json').read_text())['detections'] if d['name']=='cigarette']
    row=dict(capture=str(capture))
    try:
        if len(candidates)!=1:raise ValueError('one_cigarette_candidate_required')
        model=estimate(rgb,depth,s,candidates[0]);appearance=appearance_from_source(rgb,depth,s,model)
        near=measure_near(rgb,depth,s,model['center_m'],model,appearance)
        row.update(model=model,appearance=appearance,near=near)
    except Exception as exc:row['reason']=str(exc)
    rows.append(row)
report=dict(tests_passed=tests.testsRun,actual_source_frames=rows,robot_commands_sent=False)
with (root/'flat-box-source-analysis.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report))

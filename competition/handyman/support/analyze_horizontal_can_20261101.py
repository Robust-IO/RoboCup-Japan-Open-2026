import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/runtime')
from horizontal_can_geometry import estimate
root=Path('/__HANDYMAN_PACKAGE__/runtime/horizontal-can-source-20261101-01')
rows=[]
for path in sorted((root/'captures').iterdir()):
    detections=json.loads((path/'detections.json').read_text())['detections']
    targets=[d for d in detections if d['name']=='canned_juice']
    row=dict(capture=str(path),object='canned_juice')
    try:
        if len(targets)!=1:raise ValueError('nonunique_can_detection')
        row['measurement']=estimate(np.asarray(Image.open(path/'rgb.png').convert('RGB')),
            np.load(path/'depth_m.npy',allow_pickle=False),json.loads((path/'snapshot.json').read_text()),targets[0])
    except Exception as exc:row['reason']=str(exc)
    rows.append(row)
report=dict(observations=rows,robot_commands_sent=False,physical_grasp_verified=False)
(root/'horizontal-geometry-analysis.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))

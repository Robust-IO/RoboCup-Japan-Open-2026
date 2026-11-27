"""Offline comparison on the last successful grasp's saved current RGBD."""
import json
import sys
from pathlib import Path
import numpy as np
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from canned_grasp_geometry import measure_near
base=Path('/__HANDYMAN_PACKAGE__/runtime')
grasp=base/'can-centerline-baseline-depth-visible-20261024-07/grasp'
read=lambda p:json.loads(p.read_text());result=[]
for name in ('approach-00-before','approach-01-before','preclose'):
    capture=grasp/'captures'/name
    if not capture.exists():continue
    measured=read(capture/'can-measurement.json');snapshot=read(capture/'snapshot.json')
    row=dict(capture=str(capture),cap_axis_center_m=measured['center_m'])
    try:
        legacy=measure_near(np.load(capture/'depth_m.npy'),snapshot,measured['center_m'],
            measured['footprint_radius_m'],measured['height_m'])
        row.update(legacy_center_m=legacy['center_m'],legacy_minus_cap_axis_m=(np.asarray(legacy['center_m'])-measured['center_m']).tolist())
    except ValueError as exc:row['legacy_rejection']=str(exc)
    result.append(row)
print(json.dumps(result))

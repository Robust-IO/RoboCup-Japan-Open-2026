import json,sys,importlib.util
from pathlib import Path
import numpy as np
from PIL import Image
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
from can_destination_surface import measure
spec=importlib.util.spec_from_file_location('candidate_penguin','/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/penguin_geometry_component_candidate_20261026.py');model=importlib.util.module_from_spec(spec);spec.loader.exec_module(model)
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
table=next(t for t in json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/maps/export/LayoutA/semantic_map.json').read_text())['destinations'] if t['id']=='white_side_table#bedroom_1')
rows=[]
for f in sorted((base/'penguin-unified-transfer-20261025-03/grasp/captures').glob('identity-*')):
    s=json.loads((f/'snapshot.json').read_text());dep=np.load(f/'depth_m.npy');rgb=np.asarray(Image.open(f/'rgb.png').convert('RGB'))
    detections=[d for d in json.loads((f/'raw-detector-result.json').read_text())['detections'] if d['name']=='toy_penguin' and d['confidence']>=.8]
    if not detections:continue
    row=dict(capture=str(f))
    try:
        m=model.estimate_source(rgb,dep,s,detections[0],measured_support=measure(dep,s,table));model.validate_reference(m)
        row.update(valid=True,body_radius=m['body_radius_m'],head_radius=m['cap_radius_m'],extent=m['observed_horizontal_extent_m'],grasp_offset=m['head_grasp_axial_offset_m'])
    except Exception as e:row.update(valid=False,reason=str(e))
    rows.append(row)
result=dict(rows=rows,robot_commands_sent=False,production_geometry_unchanged=True)
p=base/'penguin-component-offline-replay-20261026.json';p.write_text(json.dumps(result,indent=2));print(json.dumps(result))

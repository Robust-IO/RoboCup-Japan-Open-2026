import json,sys
from pathlib import Path
import numpy as np
from PIL import Image
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from penguin_geometry import estimate_source,validate_reference
from can_destination_surface import measure
base=Path('/__HANDYMAN_PACKAGE__/runtime')
tables=json.loads(Path('/__HANDYMAN_PACKAGE__/maps/export/LayoutA/semantic_map.json').read_text())['destinations']
table=next(t for t in tables if t['id']=='white_side_table#bedroom_1')
rows=[]
for f in sorted((base/'penguin-unified-transfer-20261025-03/grasp/captures').glob('identity-*')):
    s=json.loads((f/'snapshot.json').read_text());dep=np.load(f/'depth_m.npy');rgb=np.asarray(Image.open(f/'rgb.png').convert('RGB'))
    dets=[d for d in json.loads((f/'raw-detector-result.json').read_text())['detections'] if d['name']=='toy_penguin' and d['confidence']>=.8]
    if not dets:continue
    m=estimate_source(rgb,dep,s,dets[0],measured_support=measure(dep,s,table))
    row=dict(capture=str(f),measurement=m)
    try:validate_reference(m);row['valid']=True
    except ValueError as e:row.update(valid=False,reason=str(e))
    rows.append(row)
p=base/'penguin-reference-source-diagnostic-20261026.json';p.write_text(json.dumps(rows,indent=2));print(json.dumps(rows))

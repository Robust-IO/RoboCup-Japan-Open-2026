import json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from canned_grasp_geometry import closure_error,transform_point,contact_point
r=Path('/__HANDYMAN_PACKAGE__/runtime/pepper-frozen-cross-object-transfer-20261026-01')
rows=[]
for f in sorted((r/'grasp/captures').glob('*-near')):
 s=json.loads((f/'snapshot.json').read_text());m=json.loads((f/'object-measurement.json').read_text())
 row=dict(capture=str(f),stamp_ns=m['stamp_ns'],center=m['center_m'],radius=m['body_radius_m'],cap=m['cap_center_m'],contact=contact_point(s).tolist(),
  world_center=transform_point(s,m['center_m']).tolist(),world_cap=transform_point(s,m['cap_center_m']).tolist(),error=closure_error(m['center_m'],s).tolist(),body_fit=m['body_fit'],cap_fit=m['cap_fit'])
 rows.append(row)
p=r/'alignment-audit.json';p.write_text(json.dumps(rows,indent=2));print(json.dumps(rows))

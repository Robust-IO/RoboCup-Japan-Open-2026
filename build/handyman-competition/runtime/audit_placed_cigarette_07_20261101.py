import json,sys
from pathlib import Path
import numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');sys.path[:0]=[str(b),'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1']
from flat_box_place_geometry import region
from can_destination_surface import measure
from source_table_front import observed_front
root=b/'cigarette-place-20261101-07';plan=json.loads((root/'placement-plan.json').read_text());cfg=json.loads((root/'config.snapshot.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==cfg['destination_id']);rows=[]
for cap in sorted((root/'captures').glob('placed-*')):
 if not (cap/'object-measurement.json').exists():continue
 s=json.loads((cap/'snapshot.json').read_text());m=json.loads((cap/'object-measurement.json').read_text());d=np.load(cap/'depth_m.npy');plane=measure(d,s,table);check=region(m,s,plan,plane)
 try:_,front=observed_front(d,s,table,plane)
 except ValueError as exc:front=dict(reason=str(exc))
 rows.append(dict(capture=str(cap),snapshot=s,measurement=m,plane=plane,check=check,front=front))
out=dict(robot_commands_sent=False,rows=rows);(root/'post-placement-audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps([dict(capture=r['capture'],check=r['check'],front=r['front']) for r in rows]))

import sys,json
from pathlib import Path
import numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from apple_transfer_auto_plan import map_point
from can_destination_surface import measure
from tabletop_support import support_from_plan
p=b/'round-place-return-20261107-171228';cap=p/'captures/placed-4';s=json.loads((cap/'snapshot.json').read_text());m=json.loads((cap/'object-measurement.json').read_text());plan=json.loads((p/'placement-plan.json').read_text());cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='dining_table#kitchen');plane=measure(np.load(cap/'depth_m.npy'),s,table);c=map_point(s,np.array(m['center_m']));bottom=map_point(s,np.r_[m['center_m'][:2],m['bottom_z_m']]);print(json.dumps(dict(center=c.tolist(),bottom=bottom.tolist(),plane=plane,gap=float(bottom[2]-np.r_[bottom[:2],1.]@plane['plane_map']),edge=support_from_plan(plan).query(c[:2],plan['radius_m']),release=json.loads((p/'release.json').read_text())),indent=2))

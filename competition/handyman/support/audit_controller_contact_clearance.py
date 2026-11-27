import sys,json
from pathlib import Path
import numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');sys.path[:0]=[str(b),'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1']
from thin_box_clearance import ThinBoxClearance
from hand_camera_ray import rotation
p=b/'controller-overhead-handle-20261107-173233';s=json.loads((p/'captures/preclose/snapshot.json').read_text());palm=s['base_from_palm_at_hand'];env=ThinBoxClearance().envelope;rows=[]
for sample in env['samples']:
 for part in sample['parts']:
  if part.get('is_trigger'):continue
  v=np.asarray(part['vertices_palm_m'])@rotation(palm['quaternion']).T+palm['translation'];gap=float(v[:,2].min()-.497);rows.append(dict(path=part['path'],geometric_bottom_clearance_m=gap,with_contact_and_uncertainty_m=gap-part['contact_offset_m']-env['environment_contact_offset_m']-env['uncertainty_bound_m']))
print(json.dumps(sorted(rows,key=lambda v:v['with_contact_and_uncertainty_m'])[:4],indent=2))
print((b/'controller-replace-latest.json').read_text() if (b/'controller-replace-latest.json').exists() else 'replace in progress')

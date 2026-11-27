"""Back up active closure code and inspect real open fingers; no ROS."""
import hashlib,json,shutil,sys,time
from pathlib import Path
import numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');sys.path[:0]=[str(b),'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1']
from thin_box_clearance import ThinBoxClearance
from horizontal_can_grasp_plan import forward
from inverse_table_place_plan import FLEX,WRIST
names=['flat_box_grasp.py','thin_box_clearance.py','test_finger_longitudinal_overlap.py','OUTCOME_TRANSFER_POLICY.txt','outcome_transfer_policy.json']
root=b/'backups'/('before-closure-region-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir()
files=[]
for name in names:
    shutil.copyfile(b/name,root/name)
    files.append(dict(name=name,sha256=hashlib.sha256((root/name).read_bytes()).hexdigest()))
(root/'manifest.json').write_text(json.dumps(files,indent=2));(b/'closure-region-backup-latest.json').write_text(json.dumps(dict(backup=str(root))))
safety=ThinBoxClearance();p,r=forward([.25,FLEX,0,WRIST,0,.65]);rows=[]
for region in safety.open_contacts:
    v=np.asarray(region['vertices_palm_m'])@r.T+p
    rows.append(dict(side=region['side'],min=v.min(0).tolist(),max=v.max(0).tolist()))
print(json.dumps(dict(backup=str(root),files=files,palm=p.tolist(),rotation=r.tolist(),open_regions_base=rows)))

import json,sys,os
from pathlib import Path
import cv2,numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),str(f)]
from apple_worker_client import Worker
from can_destination_surface import measure as table
from hand_camera_ray import rotation
from round_body_geometry import measure
r=Path(json.loads((b/'tumbler-task-latest.json').read_text())['output']);cap=r/'capture/captures/stopped-capture';cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());os.environ['HANDYMAN_RESIDENT_MODEL']='1'
w=Worker(cfg['model_python'],cfg['registry'],r/'round-model.log',imgsz=960)
raw=w.infer(cap);w.close();(r/'round-detections.json').write_text(json.dumps(raw,indent=2));print(json.dumps(raw),flush=True)
d=next(d for d in raw['detections'] if d['name']=='tumbler' and d['confidence']>=.8)
s=json.loads((cap/'snapshot.json').read_text());depth=np.load(cap/'depth_m.npy');rgb=cv2.cvtColor(cv2.imread(str(cap/'rgb.png')),cv2.COLOR_BGR2RGB)
t=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='white_side_table#kitchen');plane=table(depth,s,t);coef=plane['plane_map'];tf=s['tracking_transforms']['map_from_base_at_depth'];n=np.array([-coef[0],-coef[1],1.]);nb=rotation(tf['quaternion']).T@n;bc=np.r_[-nb[:2]/nb[2],(coef[2]-n@tf['translation'])/nb[2]]
g=measure(rgb,depth,s,box=d['box'],plane=bc);(r/'round-geometry.json').write_text(json.dumps(dict(detection=d,geometry=g,plane_base=bc.tolist()),indent=2));print(json.dumps(g))

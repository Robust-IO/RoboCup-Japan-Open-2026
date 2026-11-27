import sys,json,os
from pathlib import Path
import cv2,numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),str(f)]
from apple_worker_client import Worker
from can_destination_surface import measure
from hand_camera_ray import rotation
from round_body_geometry import grid
root=Path(json.loads((b/'toy-car-source-latest.json').read_text())['output']);cap=root/'captures/car-source';cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());os.environ['HANDYMAN_RESIDENT_MODEL']='1'
w=Worker(cfg['model_python'],cfg['registry'],root/'toy-car-model.log',imgsz=960);raw=w.infer(cap);w.close();(root/'toy-car-detections.json').write_text(json.dumps(raw,indent=2));print(json.dumps(raw),flush=True)
s=json.loads((cap/'snapshot.json').read_text());depth=np.load(cap/'depth_m.npy');T=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='white_side_table#kitchen');plane=measure(depth,s,T);c=plane['plane_map'];tf=s['tracking_transforms']['map_from_base_at_depth'];n=np.array([-c[0],-c[1],1.]);nb=rotation(tf['quaternion']).T@n;bc=np.r_[-nb[:2]/nb[2],(c[2]-n@tf['translation'])/nb[2]]
pts=grid(depth,s);z=pts[:,:,2]-(pts[:,:,:2]@bc[:2]+bc[2]);d=next(d for d in raw['detections'] if d['name']=='toy_car');x1,y1,x2,y2=map(int,d['box']);roi=np.zeros(depth.shape,bool);roi[y1:y2,x1:x2]=1;mask=roi&(depth>.1)&np.isfinite(depth)&(z>.005)&(z<.2);pp=pts[mask];np.save(root/'toy-car-points.npy',pp);(root/'toy-car-plane.json').write_text(json.dumps(dict(plane_base=bc.tolist(),plane=plane,box=d['box']),indent=2));print(json.dumps(dict(count=len(pp),min=pp.min(0).tolist(),max=pp.max(0).tolist(),quantiles=np.quantile(pp,[.01,.5,.99],axis=0).tolist())))
image=cv2.imread(str(cap/'rgb.png'));cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),2);cv2.imwrite(str(root/'toy-car-detected.png'),image)

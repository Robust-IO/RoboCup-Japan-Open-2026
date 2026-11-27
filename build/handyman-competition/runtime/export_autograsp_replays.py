import sys,json
from pathlib import Path
import numpy as np,cv2
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),str(f)]
from thin_box_clearance import ThinBoxClearance
from can_destination_surface import measure as measure_table
from local_handle_geometry import plane_base
from round_body_geometry import grid
out=b/'autograsp-v1/replay-inputs';out.mkdir(parents=True,exist_ok=True);s=ThinBoxClearance();e=s.envelope
hand=dict(sweep_vertices=[p['vertices_palm_m'] for x in e['samples'] for p in x['parts'] if not p.get('is_trigger')],sweep_offsets=[p['contact_offset_m'] for x in e['samples'] for p in x['parts'] if not p.get('is_trigger')],open_boxes=[p['vertices_palm_m'] for p in s.hand.sample(.65)['parts']],open_contacts=[p['vertices_palm_m'] for p in s.open_contacts],uncertainty_m=e['uncertainty_bound_m'],environment_offset_m=e['environment_contact_offset_m'],complete_physical_sweep_verified=True)
(out/'hand.json').write_text(json.dumps(hand))
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());T=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='white_side_table#kitchen')
rows=[('tumbler',b/'generic-round-grasp-20261107-170503/captures/identity-0'),('game_controller',b/'generic-handle-grasp-20261107-171912/captures/source'),('toy_car',b/'generic-upper-body-grasp-20261107-175004/captures/source'),('cigarette',b/'cigarette-grasp-20261101-17/captures/source')];manifest=[]
for name,cap in rows:
 snap=json.loads((cap/'snapshot.json').read_text());depth=np.load(cap/'depth_m.npy');rgb=cv2.cvtColor(cv2.imread(str(cap/'rgb.png')),cv2.COLOR_BGR2RGB);raw=cap/'detections.json'
 if not raw.exists():raw=cap.parent.parent/'detections.json'
 ds=json.loads(raw.read_text())['detections'];d=max((v for v in ds if v['name']==name),key=lambda v:v['confidence']);plane=np.array(json.loads((cap.parent.parent/'source-geometry.json').read_text())['support_plane_base']) if name=='cigarette' else plane_base(snap,measure_table(depth,snap,T)['plane_map']);points=grid(depth,snap);h=points[:,:,2]-(points[:,:,:2]@plane[:2]+plane[2]);valid=np.isfinite(depth)&(depth>.05)&(depth<3);roi=np.zeros(depth.shape,bool);x1,y1,x2,y2=map(int,d['box']);roi[y1:y2,x1:x2]=True;mask=valid&roi&(h>.006)&(h<.4);target=points[mask];lo=target.min(0)-[.2,.2,.1];hi=target.max(0)+[.2,.2,.2];env=points[valid&~mask&(points>=lo).all(2)&(points<=hi).all(2)]
 np.savez_compressed(out/(name+'.npz'),target=target,environment=env,plane=plane,stamp_ns=snap['streams']['depth']['stamp_ns']);manifest.append(dict(identity=name,input=name+'.npz',source=str(cap),depth_points=len(target),motion_executed=False))
(out/'manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(dict(output=str(out),cases=manifest,hand_sweep_boxes=len(hand['sweep_vertices']))))

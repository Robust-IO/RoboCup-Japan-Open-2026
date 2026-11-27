import sys,json
from pathlib import Path
import numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from thin_box_clearance import ThinBoxClearance
from horizontal_can_grasp_plan import forward
from table_transfer_core import FLEX,WRIST
p=b/'round-placement-recheck-20261107-171522';pts=np.load(p/'controller-points.npy');plane=np.array(json.loads((p/'controller-plane.json').read_text())['plane_base']);safety=ThinBoxClearance();_,r=forward([.25,FLEX,0,WRIST,0,.65]);env=safety.envelope
required=max(float(np.max((np.asarray(part['vertices_palm_m'])@r.T)[:,:2]@plane[:2]+plane[2]-(np.asarray(part['vertices_palm_m'])@r.T)[:,2])+part['contact_offset_m']+env['environment_contact_offset_m']+env['uncertainty_bound_m']+.005) for sample in env['samples'] for part in sample['parts'] if not part.get('is_trigger'))
print(json.dumps(dict(min_palm_height=required,plane=plane.tolist(),rotation=r.tolist(),environment_offset=env['environment_contact_offset_m'],uncertainty=env['uncertainty_bound_m'])))
for x in np.arange(np.quantile(pts[:,0],.02)+.025,np.quantile(pts[:,0],.02)+.065,.008):
 q=pts[abs(pts[:,0]-x)<.012];ys=np.sort(q[:,1]);cuts=np.r_[0,np.where(np.diff(ys)>.012)[0]+1,len(ys)]
 for l,h in zip(cuts[:-1],cuts[1:]):
  if h-l<30:continue
  lo,hi=np.quantile(ys[l:h],[.01,.99]);qq=q[(q[:,1]>=lo)&(q[:,1]<=hi)];top=np.quantile(qq[:,2],.95);bottom=float(np.r_[x,(lo+hi)/2,1.]@plane);height=top-bottom;overlap=min(top,required+.0175)-max(bottom,required-.0175)
  print(json.dumps(dict(x=x,y=(lo+hi)/2,width=hi-lo,top=top,bottom=bottom,min_palm=required,vertical_overlap=overlap,n=len(qq))))

for part in safety.open_contacts:
 v=np.asarray(part["vertices_palm_m"])@r.T;print("PAD",v.min(0).tolist(),v.max(0).tolist())

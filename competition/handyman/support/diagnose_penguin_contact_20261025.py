import json,sys
from pathlib import Path
import cv2,numpy as np
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
import penguin_geometry as g
from canned_grasp_geometry import transform_point
root=Path('/__HANDYMAN_PACKAGE__/runtime/penguin-native-transfer-20261025-03')
report=json.loads((root/'result.json').read_text());last=report['source_observations'][-1]
source=json.loads((Path(last['capture'])/'snapshot.json').read_text());ref=last['measurement']
b=transform_point(source,ref['center_m']);h=transform_point(source,ref['cap_center_m'])
original=g.fit_sphere
for name in ('008-near','009-near','010-near-rejected'):
 p=root/'grasp/captures'/name;s=json.loads((p/'snapshot.json').read_text())
 eb=transform_point(s,b,inverse=True);eh=transform_point(s,h,inverse=True);expected=iter((eb,eh));fits=[]
 def traced(points,limits):
  e=next(expected);q=points-points.mean(0);v,_,_,_=np.linalg.lstsq(np.c_[2*q,np.ones(len(q))],np.sum(q*q,axis=1),rcond=None);print(json.dumps(dict(diagnostic_only=True,part=len(fits),points=len(points),initial_radius=float(np.sqrt(v[3]+v[:3]@v[:3])),limits=limits,expected=e.tolist())));np.save('/__HANDYMAN_PACKAGE__/runtime/penguin-'+name+'-part-'+str(len(fits))+'.npy',points);center,r,proof=original(points,limits)
  fits.append(dict(center=center.tolist(),radius=r,expected=e.tolist(),error_m=float(np.linalg.norm(center-e)),proof=proof))
  return center,r,proof
 g.fit_sphere=traced
 rgb=cv2.cvtColor(cv2.imread(str(p/'rgb.png')),cv2.COLOR_BGR2RGB);depth=np.load(p/'depth_m.npy')
 try:result=g.measure_near(rgb,depth,s,eb,eh,ref);reason='passed'
 except ValueError as ex:reason=str(ex)
 print(json.dumps(dict(capture=name,reason=reason,fits=fits,current_base=s['tracking_transforms']['odom_from_base_at_depth'])))

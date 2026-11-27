import json,sys
from pathlib import Path
import cv2,numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');sys.path[:0]=[str(b),'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1']
from flat_box_geometry import appearance_from_source,measure_near
from canned_grasp_geometry import transform_point
from apple_transfer_auto_plan import depth_in_map,footprint,map_pose,map_point
from can_destination_surface import measure
root=b/'cigarette-right-shift-20261101-194019';report=json.loads((root/'result.json').read_text())
def capture(n):
 p=root/'captures'/n;return cv2.cvtColor(cv2.imread(str(p/'rgb.png')),cv2.COLOR_BGR2RGB),np.load(p/'depth_m.npy'),json.loads((p/'snapshot.json').read_text())
rgb0,d0,s0=capture('before-right-shift');rgb,d,s=capture('after-right-shift');m0=report['measurement'];appearance=appearance_from_source(rgb0,d0,s0,m0);expected=transform_point(s,transform_point(s0,m0['center_m']),inverse=True);m=measure_near(rgb,d,s,expected,m0,appearance)
cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==cfg['source_table_id']);plane=measure(d,s,table);points=depth_in_map(d,s);c,h=footprint(table);coef=np.asarray(plane['plane_map'])
residual=points[:,2]-(points[:,:2]@coef[:2]+coef[2]);on=(abs(residual)<.004)&(abs(points[:,:2]-c)<h+.5).all(1)
values=points[on];out=dict(physical_motion_performed=False,pose=map_pose(s),model_after=m,expected=expected.tolist(),appearance=appearance,plane=plane,
 visible_plane_bounds_map_m=[values.min(0).tolist(),values.max(0).tolist()],visible_plane_quantiles_map_m=np.quantile(values,[.01,.1,.5,.9,.99],axis=0).tolist(),
 source_bbox=table['axis_aligned_bounds'],target_map=map_point(s,np.asarray(m['center_m'])).tolist())
(root/'current-depth-audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))

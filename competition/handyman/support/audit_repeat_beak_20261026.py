import json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from penguin_hand_hold import yellow_mask,transform_point,rotation,HAND_RASTER_TO_FRAME
r=Path('/__HANDYMAN_PACKAGE__/runtime/penguin-resident-repeat-20261026-01/restore-object')
f=r/'grasp/captures/010-near';s=json.loads((f/'snapshot.json').read_text());m=json.loads((f/'object-measurement.json').read_text());hand=cv2.imread(str(f/'hand.png'))
h_odom=transform_point(s,m['cap_center_m']);ht=s['tracking_transforms']['base_from_odom_at_hand'];head=rotation(ht['quaternion'])@h_odom+ht['translation']
bh=s['base_from_hand'];frame=rotation(bh['quaternion']).T@(head-bh['translation']);raster=HAND_RASTER_TO_FRAME.T@frame
k=np.array(s['hand_k']).reshape(3,3);radius=m['cap_radius_m'];pixel=(k@raster)[:2]/raster[2];bound=1.5*radius*max(k[0,0],k[1,1])/(raster[2]-radius)
yy,xx=np.indices(hand.shape[:2]);mask=yellow_mask(hand);mask[(xx-pixel[0])**2+(yy-pixel[1])**2>bound**2]=0
_,_,stats,_=cv2.connectedComponentsWithStats(mask)
result=dict(robot_commands_sent=False,pixel=pixel.tolist(),bound=float(bound),allowed_component_area=[40.,float(.6*np.pi*bound**2)],yellow_components=stats[1:].tolist(),
 actual_current_image_manually_reviewed='penguin still visible between fingers after lift; this interrupted restore is not a timed repeat or automatic success',production_code_changed=False)
p=r.parent/'restore-validation-audit.json';p.write_text(json.dumps(result,indent=2));print(json.dumps(result))

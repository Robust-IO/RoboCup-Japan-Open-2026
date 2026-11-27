import sys,json,cv2,numpy as np
from pathlib import Path
sys.path[:0]=['/__HANDYMAN_PACKAGE__/runtime','/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1']
from held_can_appearance import pixel_points
from hand_camera_ray import rotation
from apple_transfer_auto_plan import footprint
from flat_box_geometry import estimate
b=Path('/__HANDYMAN_PACKAGE__/runtime');p=b/'cigarette-grasp-20261101-03/captures/source'
rgb=cv2.cvtColor(cv2.imread(str(p/'rgb.png')),cv2.COLOR_BGR2RGB);depth=np.load(p/'depth_m.npy');s=json.loads((p/'snapshot.json').read_text())
ref=json.loads((b/'cigarette-grasp-20261101-01/held-context.json').read_text());a=ref['appearance'];print('reference',ref['model'],'appearance',a)
cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());table=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']=='square_low_table#living_room')
points=pixel_points(depth,s);tf=s['tracking_transforms']['map_from_base_at_depth'];mapped=points@rotation(tf['quaternion']).T+tf['translation'];c,h=footprint(table)
hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV);dh=abs(hsv[:,:,0].astype(float)-a['hue']);mask=((np.minimum(dh,180-dh)<=a['tolerance'])&(hsv[:,:,1]>45)&(hsv[:,:,2]>30)&(depth>0)&(abs(mapped[:,:,:2]-c)<h).all(2)).astype(np.uint8)
n,l,st,cs=cv2.connectedComponentsWithStats(mask,8)
for i in range(1,n):
 x,y,w,h,area=st[i]
 if area<40:continue
 print('component',st[i].tolist(),'depth_bounds',points[l==i].min(0).tolist(),points[l==i].max(0).tolist())
 try:print('estimate',estimate(rgb,depth,s,dict(name='cigarette',box=[float(x-1),float(y-1),float(x+w+1),float(y+h+1)],confidence=None)))
 except Exception as e:print(type(e).__name__,str(e))

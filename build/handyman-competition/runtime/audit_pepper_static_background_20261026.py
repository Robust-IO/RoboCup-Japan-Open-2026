"""Read-only matched static depth landmarks vs target motion; no robot IO."""
import hashlib,json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
from hand_camera_ray import rotation
from canned_grasp_geometry import transform_point
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pepper-frozen-cross-object-transfer-20261026-01')

def load(n):
 p=base/'grasp/captures'/n
 return p,json.loads((p/'snapshot.json').read_text()),np.load(p/'depth_m.npy'),cv2.imread(str(p/'rgb.png'))

def world(s,d,xy):
 u,v=np.rint(xy).astype(int);z=float(d[v,u]);k=np.array(s['rgb_k']).reshape(3,3)
 if not np.isfinite(z) or not .05<z<4:return None
 local=d[max(0,v-1):v+2,max(0,u-1):u+2]
 if np.ptp(local)>.03:return None
 camera=np.array([(xy[0]-k[0,2])*z/k[0,0],(xy[1]-k[1,2])*z/k[1,1],z])
 tf=s['base_from_rgb'];p=rotation(tf['quaternion'])@camera+tf['translation']
 if p[2]<.40 or p[0]<.65:return None
 m=json.loads((base/'grasp/source-reference.json').read_text())
 return transform_point(s,p)

def features(s,rgb):
 mask=np.ones(rgb.shape[:2],np.uint8)*255;mask[int(rgb.shape[0]*.74):]=0
 # Exclude target/head/wrist pixels conservatively; do not classify them as static.
 k=np.array(s['rgb_k']).reshape(3,3);tf=s['base_from_rgb']
 refs=[]
 for f in (base/'grasp/captures').glob('*-near'):
  if (f/'snapshot.json').read_text()==json.dumps(s):continue
 # Target bbox from observed source and current base transform.
 own=json.loads((base/'grasp/source-reference.json').read_text())
 return cv2.SIFT_create(nfeatures=2500).detectAndCompute(cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY),mask)

rows=[]
for first,second in (('002-near','004-near'),('004-near','005-near'),('006-near','008-near'),('008-near','010-near'),('010-near','011-near'),('012-near','013-near')):
 pa,sa,da,ia=load(first);pb,sb,db,ib=load(second)
 ka,xa=features(sa,ia);kb,xb=features(sb,ib)
 matches=[m for m,n in cv2.BFMatcher().knnMatch(xa,xb,k=2) if m.distance<.65*n.distance]
 aa=[];bb=[];pixels=[]
 ma=json.loads((pa/'object-measurement.json').read_text());mb=json.loads((pb/'object-measurement.json').read_text())
 ca=transform_point(sa,ma['center_m']);cb=transform_point(sb,mb['center_m'])
 for m in matches:
  x,y=ka[m.queryIdx].pt,kb[m.trainIdx].pt
  a=world(sa,da,x);b=world(sb,db,y)
  if a is None or b is None or np.linalg.norm(a-b)>.12:continue
  if np.linalg.norm(a-ca)<.15 or np.linalg.norm(b-cb)<.15:continue
  aa.append(a);bb.append(b);pixels.append([x,y])
 a=np.asarray(aa);b=np.asarray(bb);delta=b-a
 row=dict(first=first,second=second,target_delta_odom_m=(cb-ca).tolist(),static_landmark_count=len(a))
 if len(a)>=6:
  mid=np.median(delta,axis=0);res=np.linalg.norm(delta-mid,axis=1);good=res<.02
  row.update(median_static_delta_odom_m=mid.tolist(),static_delta_residual_median_m=float(np.median(res)),
   static_landmark_inliers=int(good.sum()),static_scene_extent_m=np.ptp(a,axis=0).tolist(),
   target_relative_to_static_shift_m=(cb-ca-mid).tolist())
 rows.append(row)
result=dict(robot_commands_sent=False,pairs=rows,method='SIFT ratio-matched RGB features with locally continuous depth, target and wrist excluded; no invented scene coordinates')
p=base/'static-landmark-motion-audit.json';p.write_text(json.dumps(result,indent=2));print(json.dumps(result))

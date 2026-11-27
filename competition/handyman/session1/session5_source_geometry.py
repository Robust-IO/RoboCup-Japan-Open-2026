"""Read only the named static cupboard collider; never object spawn coordinates."""
import re,itertools,math,hashlib
import numpy as np

def extract(scene_text):
 import yaml
 from scipy.spatial.transform import Rotation
 blocks={int(i):s for i,s in re.findall(r'--- !u!\d+ &(\d+)[^\n]*\n(.*?)(?=--- !u!|\Z)',scene_text,re.S)}
 names={i:re.search(r'  m_Name: (.*)',s).group(1) for i,s in blocks.items() if s.startswith('GameObject:') and 'm_Name:' in s}
 ts={i:yaml.safe_load(s)['Transform'] for i,s in blocks.items() if s.startswith('Transform:') and 'm_Father:' in s}
 def chain(t):
  result=[]
  while t in ts:
   result.append(t);t=ts[t]['m_Father']['fileID']
  return result[::-1]
 expected='LayoutD/SIGVerseHouse/kitchen/wooden_cupboard/body'
 matches=[i for i in ts if '/'.join(names.get(ts[j]['m_GameObject']['fileID'],'?') for j in chain(i))==expected]
 if len(matches)!=1:raise ValueError('unique_session5_wooden_body_required')
 tid=matches[0];world=np.eye(4)
 for i in chain(tid):
  t=ts[i];m=np.eye(4);q=t['m_LocalRotation'];v=t['m_LocalPosition'];sc=t['m_LocalScale']
  m[:3,:3]=Rotation.from_quat([q[k] for k in 'xyzw']).as_matrix()@np.diag([sc[k] for k in 'xyz']);m[:3,3]=[v[k] for k in 'xyz'];world=world@m
 go=ts[tid]['m_GameObject']['fileID'];colliders=[]
 for i,s in blocks.items():
  if s.startswith('BoxCollider:') and f'm_GameObject: {{fileID: {go}}}' in s:
   c=yaml.safe_load(s)['BoxCollider']
   if not c.get('m_Enabled') or c.get('m_IsTrigger'):continue
   size=np.array([c['m_Size'][k] for k in 'xyz']);center=np.array([c['m_Center'][k] for k in 'xyz'])
   corners=np.array([center+np.array(sign)*size/2 for sign in itertools.product((-1,1),repeat=3)])
   pts=corners@world[:3,:3].T+world[:3,3];colliders.append((i,pts))
 if not colliders:raise ValueError('body_support_collider_missing')
 cid,points=max(colliders,key=lambda v:np.max(v[1][:,1]))
 # Unity (x,y,z) -> ROS (z,-x,y).
 ros=np.c_[points[:,2],-points[:,0],points[:,1]];lo=ros.min(0);hi=ros.max(0);center=(lo+hi)/2
 forward=world[:3,:3]@np.array([0.,0.,1.]);yaw=math.atan2(-forward[0],forward[2])
 return dict(id='body#kitchen',semantic_class='wooden_cupboard_body',room='kitchen',
  pose=dict(x=float(center[0]),y=float(center[1]),yaw=yaw),
  axis_aligned_bounds=dict(center=dict(x=float(center[0]),y=float(center[1])),size=dict(x=float(hi[0]-lo[0]),y=float(hi[1]-lo[1]))),
  front_yaw=yaw,model_path=expected,support_height_m=float(hi[2]),
  provenance=dict(source='static_scene_top_box_collider',transform_id=tid,collider_id=cid,scene_sha256=hashlib.sha256(scene_text.encode()).hexdigest(),object_truth_coordinates_used=False))

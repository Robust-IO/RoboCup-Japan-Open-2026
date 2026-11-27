"""Read-only complete hand collider/mesh inventory and palm-local bounds."""
import sys,json,copy,itertools
from pathlib import Path
import numpy as np
sys.path[:0]=['/__HANDYMAN_PACKAGE__/runtime','/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1']
from audit_gripper_contacts import parse,matrix
from horizontal_can_grasp_plan import forward
base=Path('/__HANDYMAN_PACKAGE__/runtime');prefab=Path('/__HANDYMAN_PACKAGE__/unity-reference/Assets/SIGVerse/Models/Robot/HSR/Prefabs/HSR.prefab')
objects=parse(prefab);geo=json.loads(Path('/__HANDYMAN_PACKAGE__/geometry/hsr-full-geometry-20261010-203028.json').read_text())
nodes={i:o['Transform'] for i,o in objects.items() if 'Transform' in o}
names={objects[t['m_GameObject']['fileID']]['GameObject']['m_Name']:i for i,t in nodes.items()}
palm=names['hand_palm_link']
def name(fid):return objects[nodes[fid]['m_GameObject']['fileID']]['GameObject']['m_Name']
def chain(fid):
 result=[]
 while fid and fid!=palm:result.append(fid);fid=nodes[fid]['m_Father']['fileID']
 return result if fid==palm else None
def path(fid):
 c=chain(fid)
 return '/'.join(['hand_palm_link']+[name(i) for i in reversed(c)]) if c is not None else ''
by_go={t['m_GameObject']['fileID']:i for i,t in nodes.items()};rows=[];missing=[]
for fid,o in objects.items():
 for kind in ('MeshCollider','BoxCollider','SphereCollider','CapsuleCollider'):
  if kind not in o:continue
  c=o[kind];tf=by_go.get(c['m_GameObject']['fileID'])
  if tf is None or chain(tf) is None or not c.get('m_Enabled',1) or c.get('m_IsTrigger'):continue
  rec=dict(file_id=fid,kind=kind,path=path(tf),transform=tf)
  if kind=='BoxCollider':
   center=np.array([c['m_Center'][k] for k in 'xyz']);half=np.array([c['m_Size'][k] for k in 'xyz'])/2
   rec['vertices_local']=np.array([center+np.array(s)*half for s in itertools.product((-1,1),repeat=3)]).tolist()
  elif kind=='MeshCollider':
   refs=objects[c['m_GameObject']['fileID']]['GameObject']['m_Component'];filters=[objects[r['component']['fileID']]['MeshFilter'] for r in refs if 'MeshFilter' in objects[r['component']['fileID']]]
   matches=[p for p in geo['parts'] if p['rendererPath'].endswith(path(tf))]
   if len(matches)!=1 or len(filters)!=1 or filters[0]['m_Mesh']!=c['m_Mesh']:
    missing.append(dict(rec,reason='collider_mesh_not_matched_to_exported_renderer'));continue
   part=matches[0];vertices=np.array([[v['x'],v['y'],v['z'],1.] for v in part['vertices']])
   # The exported matrix maps mesh->nearest link, so use that link's chain.
   rec.update(transform=names[part['link']],vertices_local=(vertices@np.asarray(part['meshToLinkUnityRowMajor']).reshape(4,4).T)[:,:3].tolist(),mesh_name=part['meshName'])
  else:missing.append(dict(rec,reason='unhandled_physical_collider'));continue
  rows.append(rec)
samples=[]
for opening in np.linspace(.65,0,66):
 def fk(fid):
  mat=np.eye(4)
  for i in chain(fid):
   t=copy.deepcopy(nodes[i]);angle={'hand_l_proximal_link':opening,'hand_r_proximal_link':-opening,'hand_l_distal_link':-opening,'hand_r_distal_link':opening}.get(name(i))
   if angle is not None:t['m_LocalRotation']=dict(x=float(np.sin(angle/2)),y=0,z=0,w=float(np.cos(angle/2)))
   mat=matrix(t)@mat
  return mat
 parts=[]
 for rec in rows:
  v=np.c_[rec['vertices_local'],np.ones(len(rec['vertices_local']))];p=(v@fk(rec['transform']).T)[:,:3];p[:,0]*=-1
  parts.append(dict(path=rec['path'],vertices_palm_m=p.tolist()))
 samples.append(dict(opening_rad=float(opening),parts=parts))
report=dict(source_prefab=str(prefab),source_geometry=str(Path('/__HANDYMAN_PACKAGE__/geometry/hsr-full-geometry-20261010-203028.json')),
 collider_inventory=[{k:v for k,v in r.items() if k!='vertices_local'} for r in rows],missing_colliders=missing,
 opening_samples=samples,geometry_complete_for_base_prefab=not missing,scene_instance_overrides_verified=False,robot_commands_sent=False)
output=base/'thin-box-hand-collider-audit-20261101.json';output.write_text(json.dumps(report))
summary=dict(file=str(output),colliders=len(rows),missing=missing,scene_instance_overrides_verified=False,orientations=[])
for label,q in [('horizontal',[.25,-1.7,0,.1292036732051034,0,.65]),('top_down',[.25,-np.pi/2,0,-np.pi/2,0,.65])]:
 _,r=forward(q);bounds=[]
 for sample in samples:
  verts=np.concatenate([p['vertices_palm_m'] for p in sample['parts']]);v=verts@r.T;bounds.append(float(v[:,2].min()))
 summary['orientations'].append(dict(posture=label,lowest_relative_palm_m=min(bounds),minimum_palm_above_table_for_8mm_clearance_m=.008-min(bounds)))
(base/'thin-box-hand-collider-audit-summary-20261101.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))

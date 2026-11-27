import sys,json,re
from pathlib import Path
sys.path[:0]=['/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1','/__HANDYMAN_PACKAGE__/runtime']
from audit_gripper_contacts import parse
b=Path('/__HANDYMAN_PACKAGE__/runtime');audit=json.loads((b/'thin-box-hand-collider-audit-20261101.json').read_text());objects=parse(Path(audit['source_prefab']))
ids=[r['file_id'] for r in audit['missing_colliders']];guids={objects[i]['MeshCollider']['m_Mesh']['guid'] for i in ids};assets={}
root=Path('/mnt/d/handyman/handyman-unity2/handyman-unity2/Assets/SIGVerse/Models/Robot/HSR')
for meta in root.rglob('*.meta'):
 text=meta.read_text();m=re.search(r'^guid: (.+)$',text,re.M)
 if m and m[1] in guids:assets[m[1]]=dict(path=str(meta)[:-5],metadata=text[:14000])
report=dict(missing_colliders=[dict(file_id=i,mesh=objects[i]['MeshCollider']['m_Mesh']) for i in ids],assets=assets)
(b/'thin-box-collider-asset-lookup-20261101.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))

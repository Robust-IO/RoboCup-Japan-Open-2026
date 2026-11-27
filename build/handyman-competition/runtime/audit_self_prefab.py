"""Trace robot MeshFilter assets to nearest link; report overrides, never apply masks."""
import json
import re
from pathlib import Path
import yaml
from audit_self_projection_assets import ROOT, inspect_dae

def documents(path):
    text=path.read_text(encoding='utf-8-sig')
    parts=re.split(r'^--- !u!(\d+) &(-?\d+)[^\n]*\n',text,flags=re.M)
    return [(int(parts[i]),int(parts[i+1]),yaml.safe_load(parts[i+2])) for i in range(1,len(parts),3)]

def main():
    prefab=ROOT/'Prefabs/HSR.prefab'
    docs=documents(prefab)
    gos={i:d['GameObject'] for t,i,d in docs if t==1}
    ts={i:d['Transform'] for t,i,d in docs if t==4 and 'm_LocalPosition' in d['Transform']}
    bygo={v['m_GameObject']['fileID']:i for i,v in ts.items()}
    assets={}
    for meta in (ROOT/'hsr_meshes').rglob('*.dae.meta'):
        data=yaml.safe_load(meta.read_text()); assets[data['guid']]=(meta,data)
    meshes=[]; relevant=set()
    for t,i,d in docs:
        if t!=33:continue
        f=d['MeshFilter'];go=f['m_GameObject']['fileID'];chain=[];cur=bygo.get(go);seen=set()
        while cur and cur in ts:
            if cur in seen:raise ValueError('transform cycle')
            seen.add(cur);tr=ts[cur];name=gos.get(tr['m_GameObject']['fileID'],{}).get('m_Name','?')
            chain.append(dict(transform_id=cur,name=name,position=tr['m_LocalPosition'],rotation=tr['m_LocalRotation'],scale=tr['m_LocalScale']))
            if name.endswith('_link'):break
            cur=tr['m_Father']['fileID']
        if not chain or not chain[-1]['name'].startswith(('hand_','wrist_')):continue
        relevant.update(str(c['transform_id']) for c in chain)
        asset=assets.get(f['m_Mesh'].get('guid'));asset_info=None
        if asset:
            meta,data=asset;model=data['ModelImporter'];mesh_section=model.get('meshes',{})
            asset_info=dict(path=str(meta)[:-5],guid=data['guid'],file_id=f['m_Mesh']['fileID'],
                import_settings=mesh_section,geometry=inspect_dae(Path(str(meta)[:-5])))
        meshes.append(dict(mesh_filter_id=i,mesh_reference=f['m_Mesh'],chain_to_link=chain,asset=asset_info))
    variant=ROOT.parents[3]/'Competition/Common/Prefabs/HSR-B.prefab'
    # Resolve Assets explicitly rather than assuming prefab nesting.
    variant=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/unity-reference/Assets/Competition/Common/Prefabs/HSR-B.prefab')
    overrides=[];sources=[]
    for t,i,d in documents(variant):
        if t!=1001:continue
        obj=d['PrefabInstance'];sources.append(obj.get('m_SourcePrefab'))
        for m in obj.get('m_Modification',{}).get('m_Modifications',[]):
            prop=m.get('propertyPath','')
            if str(m.get('target',{}).get('fileID')) in relevant or prop.startswith(('m_LocalScale','m_LocalPosition','m_LocalRotation')):
                overrides.append(m)
    report=dict(prefab=str(prefab),variant=str(variant),meshes=meshes,variant_sources=sources,
        transform_overrides=overrides,approved_for_runtime=False,
        limitations=['Base-prefab trace only; nested instances and scene overrides not fully resolved.',
                    'Unity imported mesh axis conversion still requires numerical/image validation.',
                    'DAE versus URDF names are not proof of geometric equality.'])
    out=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/self-prefab-audit-20261005-v1');out.mkdir(exist_ok=False)
    (out/'audit.json').write_text(json.dumps(report,indent=2))
    print('OUTPUT',out/'audit.json')
    for r in meshes:
        print(json.dumps(dict(chain=r['chain_to_link'],mesh=r['asset']['path'] if r['asset'] else r['mesh_reference']),ensure_ascii=False))
    print('VARIANT',json.dumps(sources),'OVERRIDES',len(overrides))

if __name__=='__main__':main()

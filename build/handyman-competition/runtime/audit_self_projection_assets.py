"""Read-only asset preflight for self projection. No runtime masking authority."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path('/mnt/d/handyman/handyman-unity2/handyman-unity2/Assets/SIGVerse/Models/Robot/HSR')
OUT=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/self-projection-assets-20261005-v1')
NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}

def inspect_dae(path):
    doc=ET.parse(path).getroot()
    unit=doc.find('c:asset/c:unit',NS)
    meshes=[]
    for geometry in doc.findall('c:library_geometries/c:geometry',NS):
        mesh=geometry.find('c:mesh',NS)
        sources={s.attrib['id']:s for s in mesh.findall('c:source',NS)}
        for vertices in mesh.findall('c:vertices',NS):
            position=vertices.find("c:input[@semantic='POSITION']",NS)
            src=sources[position.attrib['source'].lstrip('#')]
            accessor=src.find('c:technique_common/c:accessor',NS)
            stride=int(accessor.attrib.get('stride',1))
            offset=int(accessor.attrib.get('offset',0))
            count=int(accessor.attrib['count'])
            values=list(map(float,src.find('c:float_array',NS).text.split()))
            if stride!=3 or offset+count*stride>len(values):raise ValueError('unsupported accessor')
            points=[values[offset+i*stride:offset+i*stride+3] for i in range(count)]
            meshes.append(dict(geometry=geometry.attrib['id'],vertices=count,
                raw_bounds_min=[min(p[i] for p in points) for i in range(3)],
                raw_bounds_max=[max(p[i] for p in points) for i in range(3)]))
    nodes=[]
    for node in doc.findall('.//c:visual_scene//c:node',NS):
        nodes.append(dict(id=node.get('id'),transforms=[dict(kind=c.tag.split('}')[-1],value=c.text)
            for c in node if c.tag.split('}')[-1] in ('matrix','translate','rotate','scale')],
            instances=[x.attrib for x in node.findall('c:instance_geometry',NS)]))
    return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        unit_meter=float(unit.get('meter','1')) if unit is not None else None,
        up_axis=doc.findtext('c:asset/c:up_axis',namespaces=NS),meshes=meshes,nodes=nodes)

def main():
    urdf=ROOT/'hsr_description/robots/hsrb4s.urdf'
    doc=ET.parse(urdf).getroot()
    links=[]
    for link in doc.findall('link'):
        if not link.get('name','').startswith(('hand_','wrist_')):continue
        visuals=[]
        for visual in link.findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is None:continue
            uri=mesh.attrib['filename']
            prefix='package://hsr_meshes/meshes/'
            if not uri.startswith(prefix):raise ValueError('unexpected mesh package')
            relative=Path(uri[len(prefix):])
            exact=ROOT/'hsr_meshes'/relative
            candidate=exact if exact.exists() else exact.with_suffix('.dae')
            # A similarly named DAE is only a candidate; never silently substitute.
            visuals.append(dict(urdf_uri=uri,exact_exists=exact.exists(),
                visual_origin=visual.find('origin').attrib if visual.find('origin') is not None else {},
                mesh_scale=mesh.get('scale','1 1 1'),
                candidate=inspect_dae(candidate) if candidate.exists() and candidate.suffix=='.dae' else None,
                candidate_verified_equivalent=False))
        if visuals:links.append(dict(link=link.get('name'),visuals=visuals))
    report=dict(schema='self-projection-assets-v1',approved_for_runtime=False,
        urdf=str(urdf),urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),links=links,
        blockers=['Verify Unity prefab renderer-to-link transform and mesh import settings.',
                  'Resolve STL versus DAE geometry differences explicitly.',
                  'Apply DAE scene transforms, units and axis conventions before projection.',
                  'Capture synchronized RGB/CameraInfo/TF; old RGB alone is insufficient.',
                  'Validate occlusion and depth before any detection exclusion.'])
    OUT.mkdir(exist_ok=False)
    (OUT/'audit.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(output=str(OUT/'audit.json'),links=[dict(link=l['link'],assets=[dict(uri=v['urdf_uri'],candidate=v['candidate']['path'] if v['candidate'] else None,unit=v['candidate']['unit_meter'] if v['candidate'] else None,axis=v['candidate']['up_axis'] if v['candidate'] else None) for v in l['visuals']]) for l in links]),indent=2))

if __name__=='__main__':main()

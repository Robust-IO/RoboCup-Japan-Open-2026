"""Extract a fixed sample Moderator delivery point; never export target-object poses."""
import argparse,hashlib,json,math,re
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

def scene_point(text):
    docs={}
    for kind,ident,body in re.findall(r'--- !u!(\d+) &(\d+)[^\n]*\n(.*?)(?=\n--- !u!|\Z)',text,re.S):
        if kind in ('1','4'):docs[int(ident)]=yaml.safe_load(body)
    targets=[i for i,d in docs.items() if d.get('GameObject',{}).get('m_Name')=='DeliveryPosition']
    if len(targets)!=1:raise ValueError('unique_delivery_position_required')
    transforms={i:d['Transform'] for i,d in docs.items() if 'Transform' in d}
    ident=next(i for i,t in transforms.items() if t.get('m_GameObject',{}).get('fileID')==targets[0])
    chain=[]
    while ident:
        if ident in [i for i,t in chain]:raise ValueError('cyclic_transform')
        t=transforms[ident];chain.append((ident,t));ident=t['m_Father']['fileID']
    root=docs[chain[-1][1]['m_GameObject']['fileID']]['GameObject']
    if root['m_Name']!='Moderator' or root['m_TagString']!='Moderator':raise ValueError('delivery_root_not_moderator')
    world=np.eye(4)
    for _,t in reversed(chain):
        q=t['m_LocalRotation'];m=np.eye(4)
        m[:3,:3]=Rotation.from_quat([q[k] for k in ('x','y','z','w')]).as_matrix()@np.diag([t['m_LocalScale'][k] for k in ('x','y','z')])
        m[:3,3]=[t['m_LocalPosition'][k] for k in ('x','y','z')];world=world@m
    # Exporter convention: ROS x=Unity z, ROS y=-Unity x, ROS z=Unity y.
    u=world[:3,3];return [float(u[2]),float(-u[0]),float(u[1])]

def build(root,config):
    scene=root/'Assets/Competition/Handyman/Handyman.unity'
    task=root/'SIGVerseConfig/Handyman/EnvironmentInfo03.json'
    playback=root/'SIGVerseConfig/Handyman/AvatarMotions03.dat'
    info=json.loads(task.read_text(encoding='utf-8-sig'))
    moderator=next(x for x in info['destinationsPositions'] if x['name']=='Moderator')
    if any(abs(v)>1e-8 for k in ('position','eulerAngles') for v in moderator[k].values()):
        raise ValueError('sample_moderator_relocated_reexport_required')
    if playback.stat().st_size:raise ValueError('moving_moderator_requires_live_tracking')
    point=scene_point(scene.read_text())
    from candidate_selection import load_selector
    from perception_bridge import inside
    selector=load_selector(config,json.loads(Path(config['semantic_map']).read_text()))
    env=yaml.safe_load(Path(config['environment']).read_text())
    start=env['initial_pose'];candidates=[]
    # Moderator faces Unity +z, i.e. ROS +x in this fixed sample. Approach its front.
    for angle in (0.,-.35,.35):
        for distance in (1.05,1.2):
            x=point[0]+distance*math.cos(angle);y=point[1]+distance*math.sin(angle)
            room=next((n for n,v in env['rooms'].items() if inside(x,y,v['region'])),None)
            cell=selector.cell([x,y])
            if room and selector.valid(cell) and selector.walk[cell]:
                candidates.append(dict(pose=[x,y,math.atan2(point[1]-y,point[0]-x)],room=room))
    lengths=selector.lengths(selector.cell([start['x'],start['y']]),[selector.cell(x['pose']) for x in candidates])
    candidates=[x for x in candidates if selector.cell(x['pose']) in lengths]
    if not candidates:raise ValueError('moderator_has_no_reachable_front_staging_pose')
    sources=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in (scene,task,playback)]
    return dict(schema='handyman_fixed_sample_handover_v1',destination='Moderator',delivery_point_map_m=point,
        staging_candidates=candidates,official_distance_m=.3,estimated_center_tolerance_m=.2,
        center_basis='measured_surface_bounds_center_not_rigidbody_com',sources=sources,
        scope='Fixed Session3 sample only; moving/repositioned Moderator requires live tracking or reconfiguration. Never release gripper.')

def main():
    p=argparse.ArgumentParser();p.add_argument('--unity-root',type=Path,required=True);p.add_argument('--config',type=Path,required=True);a=p.parse_args()
    c=json.loads(a.config.read_text());c['handover']=build(a.unity_root,c)
    a.config.write_text(json.dumps(c,indent=2));print(json.dumps(c['handover'],indent=2))
if __name__=='__main__':main()

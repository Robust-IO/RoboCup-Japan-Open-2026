"""Offline prefab contact geometry. No motion authorization or collision proof."""
import argparse
import itertools
import json
import re
import numpy as np
import yaml


def parse(path):
    result={}
    for block in re.split(r'^--- ',path.read_text(),flags=re.M)[1:]:
        head,body=block.split('\n',1)
        result[int(re.search(r'&(\d+)',head)[1])]=yaml.safe_load(body)
    return result


def matrix(t):
    p=t['m_LocalPosition'];s=t['m_LocalScale'];q=t['m_LocalRotation']
    x,y,z,w=np.array([q[k] for k in 'xyzw']);n=np.linalg.norm([x,y,z,w]);x,y,z,w=np.array([x,y,z,w])/n
    m=np.eye(4);m[:3,:3]=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])@np.diag([s[k] for k in 'xyz']);m[:3,3]=[p[k] for k in 'xyz'];return m


def main():
    from pathlib import Path
    p=argparse.ArgumentParser();p.add_argument('base',type=Path);p.add_argument('variant',type=Path);p.add_argument('output',type=Path);p.add_argument('--opening',type=float,default=0.0);a=p.parse_args()
    if not np.isfinite(a.opening) or not 0<=a.opening<=1.239:raise ValueError('Opening outside inspected limits')
    objects=parse(a.base);variant=parse(a.variant);records=[];relevant=set()
    for fid,o in objects.items():
        m=o.get('MonoBehaviour',{})
        if m.get('m_Script',{}).get('guid')!='3f539bfb7c4878f4caa72d6f0c2a4885' or m.get('triggerType')!=0:continue
        g=objects[m['m_GameObject']['fileID']]['GameObject'];refs=[c['component']['fileID'] for c in g['m_Component']];relevant.update(refs)
        tf=next(i for i in refs if 'Transform' in objects[i]);collider=next(objects[i]['BoxCollider'] for i in refs if 'BoxCollider' in objects[i])
        chain=[];mat=np.eye(4)
        while tf:
            t=objects[tf]['Transform'];name=objects[t['m_GameObject']['fileID']]['GameObject']['m_Name']
            if name=='hand_palm_link':break
            relevant.add(tf);chain.append(name)
            angle={'hand_l_proximal_link':a.opening,'hand_r_proximal_link':-a.opening,
                   'hand_l_distal_link':-a.opening,'hand_r_distal_link':a.opening}.get(name)
            if angle is not None:
                t=dict(t,m_LocalRotation=dict(x=float(np.sin(angle/2)),y=0,z=0,w=float(np.cos(angle/2))))
            mat=matrix(t)@mat;tf=t['m_Father']['fileID']
        if not tf:raise ValueError('Missing palm ancestor')
        center=np.array([collider['m_Center'][k] for k in 'xyz']);size=np.array([collider['m_Size'][k] for k in 'xyz'])
        corners=np.array([list(center+np.array(sign)*size/2)+[1] for sign in itertools.product((-1,1),repeat=3)])
        vertices=(mat@corners.T).T[:,:3];vertices[:,0]*=-1
        records.append(dict(side=m['gripperType'],chain=chain,palm_ros_center=vertices.mean(0).tolist(),palm_ros_min=vertices.min(0).tolist(),palm_ros_max=vertices.max(0).tolist()))
    overrides=[];removed=[]
    for o in variant.values():
        inst=o.get('PrefabInstance',{});mod=inst.get('m_Modification',{})
        for v in mod.get('m_Modifications',[]):
            if v.get('target',{}).get('fileID') in relevant:overrides.append(v)
        for v in mod.get('m_RemovedComponents',[]):
            if v.get('fileID') in relevant:removed.append(v)
    centers=np.array([r['palm_ros_center'] for r in records])
    report=dict(opening_rad=a.opening,contact_center_separation_m=float(np.linalg.norm(centers[0]-centers[1])),contacts=records,relevant_variant_overrides=overrides,removed_relevant_components=removed,
        motion_authorized=False,limitations=['HSRSubJointTrajectory four finger joint rotations modeled; runtime mimic/physics overrides not verified.', 'Variant overrides listed, not applied.', 'Does not inspect scene-instance overrides or sweep collision.'])
    with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()

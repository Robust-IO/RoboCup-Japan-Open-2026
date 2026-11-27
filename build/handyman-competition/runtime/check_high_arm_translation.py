"""Offline arm sweep vs tabletop and RGBD. Unknown space is not free space."""
import argparse
import copy
import json
from pathlib import Path
import numpy as np
from audit_gripper_contacts import parse,matrix
from hand_camera_ray import rotation


def main():
    p=argparse.ArgumentParser();p.add_argument('prefab',type=Path);p.add_argument('geometry',type=Path);p.add_argument('capture',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    s=json.loads((a.capture/'snapshot.json').read_text());j=s['joints'];objects=parse(a.prefab)
    nodes={i:o['Transform'] for i,o in objects.items() if 'Transform' in o}
    names={objects[t['m_GameObject']['fileID']]['GameObject']['m_Name']:i for i,t in nodes.items()}
    angles={'arm_flex_link':(1,j['arm_flex_joint']),'arm_roll_link':(2,-j['arm_roll_joint']),
            'wrist_flex_link':(1,j['wrist_flex_joint']),'wrist_roll_link':(2,-j['wrist_roll_joint']),
            'head_pan_link':(2,-j['head_pan_joint']),'head_tilt_link':(1,j['head_tilt_joint']),
            'hand_l_proximal_link':(0,j['hand_motor_joint']),'hand_r_proximal_link':(0,-j['hand_motor_joint']),
            'hand_l_distal_link':(0,-j['hand_motor_joint']),'hand_r_distal_link':(0,j['hand_motor_joint'])}
    cache={names['base_link']:np.eye(4)}
    def fk(fid):
        if fid in cache:return cache[fid]
        t=copy.deepcopy(nodes[fid]);name=objects[t['m_GameObject']['fileID']]['GameObject']['m_Name']
        if name=='arm_lift_link':t['m_LocalPosition']['z']+=j['arm_lift_joint']
        if name=='torso_lift_link':t['m_LocalPosition']['z']+=j['arm_lift_joint']/2
        if name in angles:
            axis,angle=angles[name];q=[0.,0.,0.,float(np.cos(angle/2))];q[axis]=float(np.sin(angle/2));t['m_LocalRotation']=dict(zip('xyzw',q))
        cache[fid]=fk(t['m_Father']['fileID'])@matrix(t);return cache[fid]
    bounds=[]
    for part in json.loads(a.geometry.read_text())['parts']:
        v=np.array([[x['x'],x['y'],x['z'],1.] for x in part['vertices']])
        xyz=(fk(names[part['link']])@np.array(part['meshToLinkUnityRowMajor']).reshape(4,4)@v.T).T[:,:3];xyz[:,0]*=-1
        bounds.append((part['link'],xyz.min(0),xyz.max(0)))
    delta=np.array([.09336279355,-.0299586638,0.]);samples=[];table=[];labels=[];envelopes=[]
    for name,lo,hi in bounds:
        if not name.startswith(('arm_','wrist_','hand_')):continue
        endlo=lo+delta;endhi=hi+delta
        sweep_lo=np.minimum(lo,endlo);sweep_hi=np.maximum(hi,endhi)
        if sweep_hi[0]>=.5426-.03 and sweep_lo[2]<=.49602+.03:table.append(name)
        axes=[np.linspace(sweep_lo[k]-.01,sweep_hi[k]+.01,max(2,int(np.ceil((sweep_hi[k]-sweep_lo[k]+.02)/.01))+1)) for k in range(3)]
        sample=np.stack(np.meshgrid(*axes,indexing='ij'),-1).reshape(-1,3)
        samples.append(sample);labels.extend([name]*len(sample))
        envelopes.append(dict(link=name,lo=(sweep_lo-.01).tolist(),hi=(sweep_hi+.01).tolist(),
            historical_table_front_gap_m=float(.5426-(sweep_hi[0]+.01))))
    points=np.concatenate(samples);novel=np.ones(len(points),bool)
    for _,lo,hi in bounds:novel &= ~np.all((points>=lo-.01)&(points<=hi+.01),axis=1)
    points=points[novel];labels=np.asarray(labels)[novel]
    t=s['base_from_rgb'];cam=(points-np.array(t['translation']))@rotation(t['quaternion'])
    k=np.array(s['rgb_k']).reshape(3,3);proj=cam@k.T
    positive=cam[:,2]>.01;uv=np.zeros((len(points),2),int);uv[positive]=np.rint(proj[positive,:2]/proj[positive,2,None]).astype(int)
    depth=np.load(a.capture/'depth_m.npy');inside=positive&(uv[:,0]>=1)&(uv[:,0]<depth.shape[1]-1)&(uv[:,1]>=1)&(uv[:,1]<depth.shape[0]-1)
    values=np.full(len(points),np.nan);values[inside]=depth[uv[inside,1],uv[inside,0]]
    valid=inside&np.isfinite(values)&(values>0)
    free=valid&(cam[:,2]<values-.03);near=valid&(np.abs(cam[:,2]-values)<=.03);behind=valid&(cam[:,2]>values+.03)
    masks=dict(depth_free=free,near_surface=near,occluded=behind,
               outside_fov=~inside,invalid_depth=inside&~valid)
    progress=np.clip((points[:,:2]@delta[:2])/np.dot(delta[:2],delta[:2]),0,1)
    planar_distance=np.linalg.norm(points[:,:2]-progress[:,None]*delta[None,:2],axis=1)
    unknown=~valid
    def describe(mask):
        pts=points[mask];pixels=uv[mask]
        return dict(count=int(mask.sum()),base_min=pts.min(0).tolist() if len(pts) else None,
                    base_max=pts.max(0).tolist() if len(pts) else None,
                    pixel_min=pixels.min(0).tolist() if len(pts) else None,
                    pixel_max=pixels.max(0).tolist() if len(pts) else None)
    by_link={name:{key:describe(mask&(labels==name)) for key,mask in masks.items()}
             for name in sorted(set(labels))}
    # Observed depth endpoints, not guessed object identities or self-mask labels.
    questionable=near|behind
    pix=uv[questionable];z=values[questionable]
    rays=np.c_[pix,np.ones(len(pix))]@np.linalg.inv(k).T
    observed=(rays*z[:,None])@rotation(t['quaternion']).T+np.array(t['translation'])
    result=dict(motion_authorized=False,translation_m=delta.tolist(),table_overlap_links=sorted(set(table)),
      novel_aabb_samples=len(points),depth_free=int(free.sum()),near_observed_surface=int(near.sum()),
      occluded_unknown=int(behind.sum()),outside_or_invalid=int((~valid).sum()),
      outside_fov=int((~inside).sum()),invalid_depth_inside_fov=int((inside&~valid).sum()),
      unknown_outside_base_swept_circle=int(np.count_nonzero(unknown&(planar_distance>.41))),
      unknown_max_distance_to_base_path_m=float(planar_distance[unknown].max()) if unknown.any() else None,
      unknown_furthest_forward_m=float(points[unknown,0].max()) if unknown.any() else None,
      by_link=by_link,swept_envelopes=envelopes,
      questionable_pixels=[dict(uv=p.tolist(),depth_m=float(d),observed_base=xyz.tolist()) for p,d,xyz in zip(pix,z,observed)],
      limits=['Swept per-part AABBs with 1cm sampling, not continuous mesh/collider sweep.',
              'Samples inside current robot AABBs excluded; this is not validated self masking.',
              'Depth free classification uses 3cm optical-depth margin, not Euclidean clearance.',
              'Inside a planar base sweep is not proof of clearance at arm height.',
              'Historical snapshot only; unknown/near samples block motion authorization.'])
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({key:value for key,value in result.items() if key not in ('by_link','swept_envelopes','questionable_pixels')},indent=2))


if __name__=='__main__':main()

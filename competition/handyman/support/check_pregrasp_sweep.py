"""Conservative sampled mesh-AABB/table-prism screen, NOT collision certification."""
import argparse
import copy
import json
from pathlib import Path
import numpy as np
from audit_gripper_contacts import parse,matrix


def main():
    p=argparse.ArgumentParser();p.add_argument('prefab',type=Path);p.add_argument('geometry',type=Path);p.add_argument('output',type=Path);p.add_argument('--extra-lift',type=float,default=0.);p.add_argument('--check-descent',action='store_true');a=p.parse_args()
    if not np.isfinite(a.extra_lift) or not 0<=a.extra_lift<=.2:raise ValueError('Invalid extra lift')
    objs=parse(a.prefab);nodes={};names={}
    for fid,o in objs.items():
        if 'Transform' in o:
            t=o['Transform'];name=objs[t['m_GameObject']['fileID']]['GameObject']['m_Name'];nodes[fid]=t;names.setdefault(name,[]).append(fid)
    geo=json.loads(a.geometry.read_text())
    required={p['link'] for p in geo['parts']}|{'base_link','hand_l_proximal_link','hand_r_proximal_link'}
    if any(len(names.get(n,[]))!=1 for n in required):raise ValueError('Ambiguous/missing links')
    parts=[]
    for part in geo['parts']:
        v=np.array([[p['x'],p['y'],p['z'],1.] for p in part['vertices']])
        parts.append((part['link'],(np.array(part['meshToLinkUnityRowMajor']).reshape(4,4)@v.T).T))
    initial=np.array([0.,0.,-np.pi/2,0.])
    lifted=np.array([.1509516385+a.extra_lift,0.,-np.pi/2,0.])
    opened=lifted.copy();opened[3]=.65
    final=np.array([.1509516385+a.extra_lift,-1.2,-.3707963268,.65])
    rows=[]
    phases=[('lift',initial,lifted),('open',lifted,opened),('extend',opened,final)]
    if a.check_descent:
        lowered=final.copy();lowered[0]=.1009516385
        phases=[('descent',final,lowered)]
    for phase,start,end in phases:
        for fraction in np.linspace(0,1,41):
            lift,flex,wrist,opening=start+(end-start)*fraction
            rotations={'arm_flex_link':(1,flex),'wrist_flex_link':(1,wrist),
                'hand_l_proximal_link':(0,opening),'hand_r_proximal_link':(0,-opening),
                'hand_l_distal_link':(0,-opening),'hand_r_distal_link':(0,opening)}
            cache={names['base_link'][0]:np.eye(4)}
            def fk(fid):
                if fid in cache:return cache[fid]
                t=copy.deepcopy(nodes[fid]);name=objs[t['m_GameObject']['fileID']]['GameObject']['m_Name']
                if name=='arm_lift_link':t['m_LocalPosition']['z']+=float(lift)
                if name in rotations:
                    axis,angle=rotations[name];q=[0.,0.,0.,float(np.cos(angle/2))];q[axis]=float(np.sin(angle/2));t['m_LocalRotation']=dict(zip('xyzw',q))
                cache[fid]=fk(t['m_Father']['fileID'])@matrix(t);return cache[fid]
            bounds=[]
            for name,v in parts:
                xyz=(fk(names[name][0])@v.T).T[:,:3];xyz[:,0]*=-1
                bounds.append((name,xyz.min(0),xyz.max(0)))
            for advance in (0.,.2326416637):
                # Entire half-space below an expanded tabletop, conservatively
                # includes unseen table legs: may reject harmless overhangs.
                edge=.5674-advance-.03;height=.496+.03
                hits=[dict(link=n,min_xyz=lo.tolist(),max_xyz=hi.tolist()) for n,lo,hi in bounds if hi[0]>=edge and lo[2]<=height]
                rows.append(dict(phase=phase,fraction=float(fraction),base_advance_m=advance,possible_table_overlap=hits))
    result=dict(motion_authorized=False,table_source='phase5-apple-down-scene-v1; historical visible edge estimate',
        table_height_m=.496,margin_m=.03,extra_lift_m=a.extra_lift,samples_per_phase=41,rows=rows,
        limitations=['Mesh AABBs vs conservative infinite table prism, not exact collider collision.',
        'Sampled path, not continuous sweep; scene overrides and other obstacles unverified.',
        'No robot self-collision check; geometry screen alone does not authorize motion.'])
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    for advance in (0.,.2326416637):
        for phase,_,_ in phases:
            selected=[r for r in rows if r['phase']==phase and r['base_advance_m']==advance]
            hits=[r for r in selected if r['possible_table_overlap']]
            print(json.dumps(dict(advance=advance,phase=phase,flagged_samples=len(hits),first=hits[0] if hits else None)))


if __name__=='__main__':main()

"""Offline pose candidates using observed surface, NOT executable grasp goals."""
import argparse
import copy
import json
import math
from pathlib import Path
import numpy as np
from audit_gripper_contacts import parse, matrix


def main():
    p=argparse.ArgumentParser()
    p.add_argument('prefab',type=Path);p.add_argument('geometry',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--full-path',action='store_true')
    p.add_argument('--full-body-screen',action='store_true')
    a=p.parse_args()
    objects=parse(a.prefab);nodes={};names={};duplicates=set()
    for fid,o in objects.items():
        if 'Transform' in o:
            t=o['Transform'];nodes[fid]=t
            name=objects[t['m_GameObject']['fileID']]['GameObject']['m_Name']
            if name in names:duplicates.add(name)
            names[name]=fid
    parts=[]
    for part in json.loads(a.geometry.read_text())['parts']:
        if part['link'] in duplicates or 'base_link' in duplicates:raise ValueError('Ambiguous required link')
        v=np.array([[p['x'],p['y'],p['z'],1] for p in part['vertices']])
        parts.append((part['link'],(np.array(part['meshToLinkUnityRowMajor']).reshape(4,4)@v.T).T))
    if a.full_body_screen:
        present={name for name,_ in parts}
        if not {'base_link','torso_lift_link','head_tilt_link','head_pan_link'}<=present:
            raise ValueError('Full body screen requires body/head geometry')

    def overlaps(lift,flex,wrist,advance,opening=.65,self_check=False):
        angles={'arm_flex_link':(1,flex),'wrist_flex_link':(1,wrist),
                'hand_l_proximal_link':(0,opening),'hand_r_proximal_link':(0,-opening),
                'hand_l_distal_link':(0,-opening),'hand_r_distal_link':(0,opening)}
        if a.full_body_screen:angles.update(head_pan_link=(2,0.),head_tilt_link=(1,-.5))
        cache={names['base_link']:np.eye(4)}
        def fk(fid):
            if fid in cache:return cache[fid]
            t=copy.deepcopy(nodes[fid]);name=objects[t['m_GameObject']['fileID']]['GameObject']['m_Name']
            if name=='arm_lift_link':t['m_LocalPosition']['z']+=float(lift)
            if a.full_body_screen and name=='torso_lift_link':t['m_LocalPosition']['z']+=float(lift)/2
            if name in angles:
                axis,angle=angles[name];q=[0.,0.,0.,math.cos(angle/2)];q[axis]=math.sin(angle/2)
                t['m_LocalRotation']=dict(zip('xyzw',q))
            cache[fid]=fk(t['m_Father']['fileID'])@matrix(t);return cache[fid]
        hits=[];bounds={};part_bounds=[]
        for name,v in parts:
            xyz=(fk(names[name])@v.T).T[:,:3];xyz[:,0]*=-1
            lo,hi=xyz.min(0),xyz.max(0)
            part_bounds.append((name,lo,hi))
            if name in bounds:
                old_lo,old_hi=bounds[name];bounds[name]=(np.minimum(lo,old_lo),np.maximum(hi,old_hi))
            else:bounds[name]=(lo,hi)
            if hi[0]>=.5426-advance-.03 and lo[2]<=.49602+.03:hits.append(name)
        if self_check:
            # Screening selected non-adjacent pairs only; anatomical neighbors
            # overlap by design. AABB overlap is NOT mesh intersection proof.
            proximal=('arm_lift_link','arm_flex_link')
            distal=('hand_palm_link','hand_l_spring_proximal_link','hand_r_spring_proximal_link','hand_l_distal_link','hand_r_distal_link')
            flags=[]
            for first in proximal:
                for second in distal:
                    if first not in bounds or second not in bounds:raise ValueError('Missing self-screen mesh')
                    a0,a1=bounds[first];b0,b1=bounds[second]
                    if np.all(np.minimum(a1,b1)-np.maximum(a0,b0)>.001):flags.append(first+' vs '+second)
            if a.full_body_screen:
                body=[v for v in part_bounds if v[0].startswith(('head_','base_')) or v[0]=='torso_lift_link']
                moving=[v for v in part_bounds if v[0].startswith(('hand_','wrist_')) or v[0] in ('arm_flex_link','arm_roll_link')]
                for first,a0,a1 in body:
                    for second,b0,b1 in moving:
                        if np.all(np.minimum(a1,b1)-np.maximum(a0,b0)>.001):flags.append(first+' vs '+second)
            return flags
        return sorted(set(hits))

    candidates=[]
    for flex in np.linspace(-1.2,-2.3,12):
        angle=-flex;wrist=-math.pi/2-flex
        lift=.5472-.339254677+math.sin(angle)*.005102036-math.cos(angle)*.34923768+.012
        palm_x=.141+math.cos(angle)*.005102036+math.sin(angle)*.34923768+.1405
        advance=.92267-(palm_x+.06212587)
        if not 0<=lift<=.69 or not -1.919<=wrist<=1.221:continue
        # Screen descent from palm height .72 to .5472 after hypothetical
        # advance. Other phases, base motion and target center are unresolved.
        hits=[]
        for f in np.linspace(0,1,41):
            links=overlaps(lift+(.72-.5472)*(1-f),flex,wrist,advance)
            if links:hits.append(dict(fraction=float(f),links=links))
        candidates.append(dict(flex=float(flex),wrist=float(wrist),lift=float(lift),
            hypothetical_base_advance_m=float(advance),lateral_alignment_needed_m=.10533-.078,
            descent_flagged_samples=len(hits),first_overlap=hits[0] if hits else None))
    result=dict(source='phase5-table-rgbd-20261010-v1',motion_authorized=False,
        table_height_m=.49602,visible_edge_proxy_m=.5426,margin_m=.03,candidates=candidates,
        limitations=['Surface point is not object center; candidate contact placement is only provisional.',
        'Sampled mesh AABB vs infinite table prism, not full collision checking.',
        'Does not validate extension, base advance, self-collision, people or robot base clearance.',
        'Base advances and lateral shift are hypothetical, not issued commands.'])
    if a.full_path:
        candidate=next(c for c in candidates if abs(c['flex']+1.7)<1e-6)
        high=candidate['lift']+.72-.5472
        flex=candidate['flex'];wrist=candidate['wrist'];advance=candidate['hypothetical_base_advance_m']
        # Use independent progress grids: Unity clamps individual joint speeds,
        # so simultaneous commands need not follow a straight joint-space line.
        phases={
          'lift_at_current_base':[(high*f,0.,-math.pi/2,0.,0.) for f in np.linspace(0,1,41)],
          'open_at_current_base':[(high,0.,-math.pi/2,0.,.65*f) for f in np.linspace(0,1,41)],
          'extend_independent_joint_grid':[(high,flex*f,-math.pi/2+(wrist+math.pi/2)*g,0.,.65)
                                          for f in np.linspace(0,1,31) for g in np.linspace(0,1,31)],
          'alternative_wrist_first':[(high,0.,-math.pi/2+(wrist+math.pi/2)*f,0.,.65) for f in np.linspace(0,1,61)],
          'alternative_flex_second':[(high,flex*f,wrist,0.,.65) for f in np.linspace(0,1,61)],
          'hypothetical_advance':[(high,flex,wrist,advance*f,.65) for f in np.linspace(0,1,41)],
          'hypothetical_descent':[(high-(high-candidate['lift'])*f,flex,wrist,advance,.65) for f in np.linspace(0,1,41)]}
        summaries={}
        for name,poses in phases.items():
            flags=[]
            for i,pose in enumerate(poses):
                links=overlaps(*pose)
                if links:flags.append(dict(index=i,joints_and_advance=pose,links=links))
            summaries[name]=dict(samples=len(poses),flagged=len(flags),first=flags[0] if flags else None)
        result['selected_candidate']=candidate
        result['high_lift_m']=high
        result['full_path_table_screen']=summaries
        self_summaries={}
        for name,poses in phases.items():
            if name=='extend_independent_joint_grid':continue
            flags=[]
            for i,pose in enumerate(poses):
                pairs=overlaps(*pose,self_check=True)
                if pairs:flags.append(dict(index=i,pairs=pairs))
            self_summaries[name]=dict(samples=len(poses),flagged=len(flags),first=flags[0] if flags else None)
        result['selected_nonadjacent_self_aabb_screen']=self_summaries
        result['limitations'].append('Self screen only arm lift/flex vs palm/fingers. Head, torso, base and all other pairs are not certified; AABBs can false-positive.')
        if a.full_body_screen:
            result['limitations'][-1]='Includes body/head vs distal arm rendered-mesh AABBs; excludes arm-lift mounting contacts. Not collider or exact mesh intersection certification; scene link-transform overrides unverified.'
            result['head_pose_assumed']={'pan':0.,'tilt':-.5}
        result['limitations'][2]='Table-only sampled path screen, not self-collision, people, full environment or robot base clearance validation.'
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

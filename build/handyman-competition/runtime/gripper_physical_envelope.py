"""Recompose scene-instance collider bounds through finger opening.

The raw export remains unapproved. This builds a geometric model and validates
the captured reference transform; it does not authorize a robot movement.
"""
import copy,math
from pathlib import Path
import numpy as np
from audit_gripper_contacts import matrix,parse

MIRROR=np.diag([-1.,1.,1.,1.])
JOINT_SIGNS={'hand_l_proximal_link':1.,'hand_r_proximal_link':-1.,
             'hand_l_distal_link':-1.,'hand_r_distal_link':1.,'hand_motor_dummy_link':1.}

class PhysicalHand:
    def __init__(self,report,*,attached_object):
        if report.get('schema')!='handyman-gripper-physical-envelope-v1' or not report.get('playMode'):
            raise ValueError('live_scene_physical_envelope_required')
        self.report=report;self.links={x['name']:x for x in report['links']}
        if len(self.links)!=len(report['links']):raise ValueError('duplicate_hand_links')
        self.by_path={x['path']:x for x in report['links']};self.excluded=[];self.parts=[]
        for i,p in enumerate(report['parts']):
            if p['isTrigger']:continue
            if attached_object and p['path'].endswith('/hand_palm_link/'+attached_object):
                self.excluded.append(p['path']);continue
            v=np.array([[x[k] for k in 'xyz'] for x in p['boundsVerticesLinkUnity']],float)
            if v.shape!=(8,3) or not np.isfinite(v).all() or p['link'] not in self.links:raise ValueError('invalid_collider_bound')
            self.parts.append(dict(index=i,path=p['path'],link=p['link'],vertices=v,contact_offset_m=p['contactOffset']))
        if len(self.parts)+len(self.excluded)!=report['activePhysicalColliderCount'] or len(self.excluded)!=(1 if attached_object else 0):
            raise ValueError('physical_collider_inventory_mismatch')
        q=self.links['hand_l_proximal_link']['localRotation'];self.reference_opening=2*math.atan2(q['x'],q['w'])
        actual=self.transforms(self.reference_opening)
        errors=[]
        for p in self.parts:
            reference=np.asarray(report['parts'][p['index']]['linkToPalmUnityRowMajor']).reshape(4,4)
            errors.append(float(np.max(abs(actual[p['link']]-reference))))
        self.reference_error=max(errors)
        if self.reference_error>2e-5:raise ValueError('scene_link_recomposition_disagrees_with_export')

    def transforms(self,opening,wrist_roll=0.):
        if not 0<=opening<=.65:raise ValueError('opening_outside_reviewed_range')
        cache={'wrist_flex_link':np.eye(4)}
        def fk(name):
            if name in cache:return cache[name]
            item=self.links[name];parent=self.by_path[item['parentPath']]['name']
            q=copy.deepcopy(item['localRotation']);angle=None
            if name in JOINT_SIGNS:angle=JOINT_SIGNS[name]*opening;axis='x'
            elif name=='wrist_roll_link':angle=-wrist_roll;axis='z'
            if angle is not None:q=dict(x=0.,y=0.,z=0.,w=float(math.cos(angle/2)));q[axis]=float(math.sin(angle/2))
            t=dict(m_LocalPosition=item['localPosition'],m_LocalScale=item['localScale'],m_LocalRotation=q)
            cache[name]=fk(parent)@matrix(t);return cache[name]
        for name in self.links:fk(name)
        inverse=np.linalg.inv(cache['hand_palm_link'])
        return {name:inverse@value for name,value in cache.items()}

    def sample(self,opening,wrist_roll=0.):
        tf=self.transforms(opening,wrist_roll);parts=[]
        for p in self.parts:
            v=(np.c_[p['vertices'],np.ones(8)]@tf[p['link']].T@MIRROR)[:,:3]
            parts.append(dict(path=p['path'],link=p['link'],vertices_palm_m=v.tolist(),is_trigger=False,contact_offset_m=p['contact_offset_m']))
        return dict(opening_rad=float(opening),parts=parts)

    def contact_regions(self,prefab,opening,wrist_roll=0.):
        objects=parse(Path(prefab));actual=self.transforms(opening,wrist_roll);rows=[]
        for obj in objects.values():
            m=obj.get('MonoBehaviour',{})
            if m.get('m_Script',{}).get('guid')!='3f539bfb7c4878f4caa72d6f0c2a4885' or m.get('triggerType')!=0:continue
            g=objects[m['m_GameObject']['fileID']]['GameObject'];refs=[x['component']['fileID'] for x in g['m_Component']]
            fid=next(i for i in refs if 'Transform' in objects[i]);c=next(objects[i]['BoxCollider'] for i in refs if 'BoxCollider' in objects[i])
            mat=np.eye(4)
            while True:
                t=objects[fid]['Transform'];name=objects[t['m_GameObject']['fileID']]['GameObject']['m_Name']
                if name.endswith('_link'):break
                mat=matrix(t)@mat;fid=t['m_Father']['fileID']
            center=np.array([c['m_Center'][k] for k in 'xyz']);half=np.array([c['m_Size'][k] for k in 'xyz'])/2
            vertices=np.array([[* (center+np.array([x,y,z])*half),1.] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
            v=(vertices@(actual[name]@mat).T@MIRROR)[:,:3]
            rows.append(dict(side=m['gripperType'],vertices_palm_m=v.tolist(),source='inspected_prefab_trigger_with_scene_instance_link_transforms'))
        if len(rows)!=2:raise ValueError('two_contact_regions_required')
        return rows

    def envelope(self,prefab,*,contact_opening,wrist_roll=0.,step=.01):
        n=int(math.ceil(.65/step));values=np.linspace(0.,.65,n+1);samples=[self.sample(x,wrist_roll) for x in values]
        # Analytic arc-length bound: proximal radius <= link-chain length plus
        # link-local point radius; distal adds its own point radius. A nearest
        # sampled angle is at most half a step away.
        bound=0.
        for p in self.parts:
            radius=float(np.linalg.norm(p['vertices'],axis=1).max())
            if 'distal' in p['link']:lipschitz=.07+2*radius
            elif 'spring_proximal' in p['link']:lipschitz=radius
            else:lipschitz=0.
            bound=max(bound,lipschitz*(.65/n)/2)
        return dict(all_physical_colliders_included=True,missing_colliders=[],instance_verified=True,
            opening_sweep_bounded=True,interval_motion_bound_m=bound,
            uncertainty_bound_m=.006+bound,environment_contact_offset_m=self.report['maximumEnvironmentContactOffset'],
            reference_transform_max_error=self.reference_error,reference_opening_rad=self.reference_opening,
            excluded_attached_object_paths=self.excluded,samples=samples,
            contact_regions=self.contact_regions(prefab,contact_opening,wrist_roll),motion_authorized=False)

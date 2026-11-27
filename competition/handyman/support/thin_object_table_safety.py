"""Thin-object pinch height from complete physical gripper envelopes.

No controller imports. A Trigger or render mesh alone is not a clearance model.
The caller supplies the complete opening sweep for the selected wrist posture.
"""
import numpy as np

THIN_HEIGHT_M=.035
GEOMETRIC_MARGIN_M=.005

def plan(model,plane,palm_xy,orientation,envelope):
    if (not envelope.get('all_physical_colliders_included') or envelope.get('missing_colliders')
            or not envelope.get('opening_sweep_bounded') or not envelope.get('instance_verified')):
        raise ValueError('complete_instance_physical_collider_sweep_required')
    p=np.asarray(plane,float);xy=np.asarray(palm_xy,float);r=np.asarray(orientation,float)
    if p.shape!=(3,) or xy.shape!=(2,) or r.shape!=(3,3) or not np.isfinite([*p,*xy,*r.ravel()]).all():raise ValueError('finite_table_and_palm_geometry_required')
    if not np.allclose(r.T@r,np.eye(3),atol=1e-5):raise ValueError('rotation_required')
    environment_offset=float(envelope['environment_contact_offset_m'])
    uncertainty=float(envelope['uncertainty_bound_m'])
    if not np.isfinite([environment_offset,uncertainty]).all() or min(environment_offset,uncertainty)<0:raise ValueError('nonnegative_contact_and_uncertainty_required')
    required=-np.inf;lowest=None
    for sample in envelope['samples']:
        for part in sample['parts']:
            if part.get('is_trigger'):continue
            vertices=np.asarray(part['vertices_palm_m'],float);offset=float(part['contact_offset_m'])
            if vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or not np.isfinite(vertices).all() or offset<0 or not np.isfinite(offset):raise ValueError('physical_collider_vertices_required')
            v=vertices@r.T
            # table_z at every collider point, including the sloped tabletop.
            floor=(v[:,:2]+xy)@p[:2]+p[2]-v[:,2]
            value=float(floor.max()+offset+environment_offset+uncertainty+GEOMETRIC_MARGIN_M)
            if value>required:required=value;lowest=part['path']
    if not np.isfinite(required):raise ValueError('physical_hand_envelope_empty')
    object_bottom=float(np.r_[model['center_m'][:2],1.]@p);height=float(model['height_m'])
    if not .003<=height<=THIN_HEIGHT_M:raise ValueError('thin_object_height_required')
    overlaps=[];upper=[]
    for trigger in envelope['contact_regions']:
        v=np.asarray(trigger['vertices_palm_m'],float)@r.T
        lo,hi=float(v[:,2].min()+required),float(v[:,2].max()+required)
        overlaps.append(max(0.,min(hi,object_bottom+height)-max(lo,object_bottom)))
        upper.append(object_bottom+height-.003-float(v[:,2].min()))
    if len(overlaps)!=2:raise ValueError('two_real_finger_contact_regions_required')
    possible=min(overlaps)>=.003
    return dict(strategy='thin_object_physical_clearance_pinch',palm_height_m=required,
        minimum_allowed_palm_height_m=required,lowering_below_this_height_forbidden=True,
        maximum_palm_height_for_contact_m=min(upper),
        grip_at_object_center_required=False,minimum_contact_overlap_m=min(overlaps),
        feasible=possible,requires_different_posture=not possible,limiting_collider=lowest,
        geometric_margin_m=GEOMETRIC_MARGIN_M,environment_contact_offset_m=environment_offset,
        uncertainty_bound_m=uncertainty,motion_authorized=False,
        reason='safe_pinch_overlap' if possible else 'no_side_overlap_without_table_contact')

def require_plan(plan,palm_height):
    if not plan.get('feasible') or not np.isfinite(palm_height) or palm_height<plan['minimum_allowed_palm_height_m']:
        raise ValueError('thin_object_no_table_contact_height_limit')
    return True

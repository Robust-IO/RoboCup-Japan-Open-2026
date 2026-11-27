"""Held delivery for the sample's distance-plus-still-grasped scoring rule."""
import numpy as np
from grasp_runtime.autograsp.kinematics import forward

def held_center(context):
    if context.get('frame')!='hand_palm_link':raise ValueError('held_palm_frame_required')
    vertices=np.asarray(context['vertices_palm_m'],float)
    if vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or not np.isfinite(vertices).all():raise ValueError('invalid_held_surface')
    return (vertices.min(0)+vertices.max(0))/2

def presenting_joints(context,height,opening):
    center=held_center(context)
    # Horizontal hand, allowing the elbow to remain slightly raised for tall delivery points.
    for flex in (-np.pi/2,-1.4,-1.2,-1.):
        joints=[0.,flex,0.,-np.pi/2-flex,0.,opening]
        p,r=forward(joints);lift=float(height-(p+r@center)[2])
        if 0<=lift<=.69:
            joints[0]=lift;return joints
    raise ValueError('delivery_height_outside_arm_reach')

def delivery_evidence(center,point,hold,*,target,tolerance=.2,require_visual=False):
    c=np.asarray(center,float);p=np.asarray(point,float)
    if c.shape!=(3,) or p.shape!=(3,) or not np.isfinite([c,p]).all():raise ValueError('invalid_delivery_geometry')
    if not 0<tolerance<.3:raise ValueError('delivery_estimate_requires_margin')
    distance=float(np.linalg.norm(c-p))
    held=hold.get('holding_verified') is True and (not require_visual or hold.get('visual_confirmation') is True)
    return dict(status='ok',target=target,destination='Moderator',holding_verified=held,visual_confirmation=hold.get('visual_confirmation') is True,
        delivery_ready=held and distance<=tolerance,
        estimated_object_center_map_m=c.tolist(),delivery_point_map_m=p.tolist(),estimated_distance_m=distance,
        center_basis='measured_surface_bounds_center_not_rigidbody_com',release_sent=False,official_success=False)

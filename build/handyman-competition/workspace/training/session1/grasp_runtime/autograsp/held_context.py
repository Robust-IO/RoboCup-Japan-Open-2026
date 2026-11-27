"""Serialize measured held surface in palm coordinates; no ROS or movement."""
import numpy as np
from grasp_runtime.autograsp.grip_confirmation import accepted


def build_context(*, points_base, palm_translation_base, palm_rotation_base,
                  stamp_ns, target, source_table, joints, lift_evidence, surface_rgb=None):
    points=np.asarray(points_base,float);t=np.asarray(palm_translation_base,float)
    r=np.asarray(palm_rotation_base,float);q=np.asarray(joints,float)
    if (points.ndim!=2 or points.shape[1]!=3 or len(points)==0
            or t.shape!=(3,) or r.shape!=(3,3) or q.shape!=(6,)
            or not all(np.isfinite(x).all() for x in (points,t,r,q))):
        raise ValueError('invalid_measured_held_geometry')
    if not np.allclose(r.T@r,np.eye(3),atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5:
        raise ValueError('invalid_palm_rotation')
    if not isinstance(stamp_ns,int) or isinstance(stamp_ns,bool) or stamp_ns<=0:
        raise ValueError('measured_depth_stamp_required')
    if not accepted(lift_evidence):
        raise ValueError('verified_lift_required')
    colours=None if surface_rgb is None else np.asarray(surface_rgb,float)
    if colours is not None and (colours.shape!=points.shape or not np.isfinite(colours).all() or (colours<0).any() or (colours>255).any()):
        raise ValueError('invalid_surface_colours')
    vertices=(points-t)@r
    return dict(schema='handyman_held_surface_v1',target=target,source_table=source_table,
                frame='hand_palm_link',depth_stamp_ns=stamp_ns,
                vertices_palm_m=vertices.tolist(),surface_rgb=None if colours is None else colours.tolist(),holding_opening_rad=float(q[5]),
                joints_at_export=q.tolist(),lift_evidence=dict(lift_evidence),
                geometry_source='last_verified_lift_observed_surface',
                complete_surface_verified=False,carry_ready=False,release_authorized=False,
                requires_current_hold_recheck=True)

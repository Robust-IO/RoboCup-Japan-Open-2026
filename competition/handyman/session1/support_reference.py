"""Infer a held object's support level from its measured pre-lift table.

This is a support-plane estimate, not a scan of an invisible underside. It is
usable only while the original support normal remains vertical at release.
"""
import math
import numpy as np


def make_reference(points_base,plane_base,palm_translation,palm_rotation,stamp_ns):
    p=np.asarray(points_base,float);plane=np.asarray(plane_base,float)
    t=np.asarray(palm_translation,float);r=np.asarray(palm_rotation,float)
    if (p.ndim!=2 or p.shape[1]!=3 or not len(p) or plane.shape!=(3,)
            or t.shape!=(3,) or r.shape!=(3,3) or not all(np.isfinite(v).all() for v in (p,plane,t,r))):
        raise ValueError('measured_source_support_required')
    if not np.allclose(r.T@r,np.eye(3),atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5:
        raise ValueError('invalid_palm_rotation')
    residual=p[:,2]-np.c_[p[:,:2],np.ones(len(p))]@plane
    if not -.01<=float(residual.min())<=.03:raise ValueError('source_contact_band_not_observed')
    normal=np.r_[-plane[:2],1.];normal/=np.linalg.norm(normal)
    xy=np.median(p[:,:2],axis=0);point=np.r_[xy,np.r_[xy,1.]@plane]
    return dict(point_palm_m=((point-t)@r).tolist(),normal_palm=(normal@r).tolist(),
                source_depth_stamp_ns=int(stamp_ns),basis='measured_pre_lift_support_plane',
                underside_directly_observed=False)


def assess_bottom(context,release_rotation):
    ref=context.get('source_support_reference')
    if not ref:return dict(established=False,reason='source_support_reference_missing')
    r=np.asarray(release_rotation,float);normal=r@np.asarray(ref['normal_palm'])
    if not np.isfinite(normal).all() or abs(np.linalg.norm(normal)-1)>1e-5:
        return dict(established=False,reason='invalid_support_normal')
    tilt=math.acos(float(np.clip(normal[2],-1,1)))
    if tilt>math.radians(.5):
        return dict(established=False,reason='release_orientation_changes_supported_bottom',tilt_rad=tilt)
    point=r@np.asarray(ref['point_palm_m'])
    vertices=np.asarray(context['vertices_palm_m'])@r.T
    # Conservative allowance for the small remaining plane tilt over observed
    # horizontal extent. No fixed apple/bottle height is used.
    radius=float(np.linalg.norm(vertices[:,:2]-point[:2],axis=1).max())
    return dict(established=True,bottom_below_palm_m=float(-point[2]+radius*math.tan(tilt)),
                basis=ref['basis'],underside_directly_observed=False,tilt_rad=tilt)

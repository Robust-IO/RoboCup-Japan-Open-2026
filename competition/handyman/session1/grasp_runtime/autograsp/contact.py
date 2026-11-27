"""Actual capture-time finger region, rather than a hard-coded centre error."""
import numpy as np


def contact_fit(points,palm,rotation,plane,hand,policy,*,require_pad_overlap=True):
    points=np.asarray(points);palm=np.asarray(palm);rotation=np.asarray(rotation)
    local=(points-palm)@rotation
    bounds=np.stack([hand.contacts.min(1),hand.contacts.max(1)],axis=1)
    lo=bounds[:,0].max(0);hi=bounds[:,1].min(0)
    left=float(bounds[:,1,1].min())+policy.side_margin_m
    right=float(bounds[:,0,1].max())-policy.side_margin_m
    band=(local[:,0]>=lo[0])&(local[:,0]<=hi[0])&(local[:,2]>=lo[2])&(local[:,2]<=hi[2])
    # Use the local contact band, so a second remote handle is not required to
    # fit between the fingers. Open-hand collision checks handle that geometry.
    band&=(local[:,1]>left-.015)&(local[:,1]<right+.015)
    q=local[band]
    if len(q)<8:return dict(fits=False,reason='contact_band_unobserved')
    width=np.quantile(q[:,1],[.03,.97])
    if width[0]<left or width[1]>right:return dict(fits=False,reason='material_outside_actual_open_gap')
    spans=np.ptp(q[:,[0,2]],axis=0)
    if require_pad_overlap and spans.min()<policy.min_contact_overlap_m:return dict(fits=False,reason='insufficient_actual_pad_overlap')
    hist=np.histogram(q[:,1],bins=6,range=tuple(width))[0]
    if width[1]-width[0]<.008 or np.count_nonzero(hist)<4 or not hist[2:4].any():
        return dict(fits=False,reason='empty_or_insufficient_actual_contact_region')
    if getattr(policy,'template_family','generic')=='neck_bottle':
        # Check the untrimmed insertion-depth extent of this local neck, not
        # only the sliver already inside the pad-depth interval.
        neck=local[(local[:,0]>=lo[0])&(local[:,0]<=hi[0])&
                   (local[:,1]>=left)&(local[:,1]<=right)]
        if len(neck)<8:return dict(fits=False,reason='neck_wrap_unobserved')
        near,far=np.quantile(neck[:,2],[.05,.95])
        if near<lo[2]-.003 or far>hi[2]+.003:
            return dict(fits=False,reason='neck_not_enclosed_in_pad_depth',
                        observed_depth_extent_m=[float(near),float(far)],pad_depth_m=[float(lo[2]),float(hi[2])])
    from .table_clearance import hand_table_clearance
    clear=hand_table_clearance(palm,rotation,plane,hand)
    if clear<policy.table_margin_m-1e-6:return dict(fits=False,reason='actual_closing_hand_too_low',clearance_m=clear)
    return dict(fits=True,reason='material_inside_actual_finger_region',clearance_m=clear,
                pad_extent_xz_m=spans.tolist(),observed_points=len(q),pad_overlap_check_enabled=require_pad_overlap)

def small_height_correction(fit,margin):
    if fit.get('reason')!='actual_closing_hand_too_low':return None
    deficit=margin-float(fit.get('clearance_m',float('nan')))
    if not np.isfinite(deficit) or not 0<deficit<=.002:return None
    return deficit+.001

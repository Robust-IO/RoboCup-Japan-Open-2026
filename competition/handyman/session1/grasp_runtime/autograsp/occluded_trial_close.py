"""Permit a trial closure when current registration aligns a hidden contact band.

This is permission to try, never proof that an object is held.
"""
import numpy as np


def decide(observed,reference,measurement,stamp,error):
    if observed.get('reason')!='contact_band_unobserved':return observed
    if not measurement or measurement.get('depth_measured') is not True or measurement.get('stamp_ns')!=stamp:return observed
    delta=np.asarray(error,float)
    if delta.shape!=(3,) or not np.isfinite(delta).all() or np.any(abs(delta)>[.009,.006,.006]):return observed
    if reference.get('fits') is not True:return observed
    return dict(reference,fits=True,reason='aligned_current_registration_trial_close',
        contact_basis='registered_reference_with_current_visual_alignment',
        current_target_depth_measured=True,current_contact_band_observed=False,
        trial_close=True,grasp_verified=False,original_rejection=observed['reason'],
        alignment_error_m=delta.tolist(),registration_stamp_ns=stamp)


def session6_depth_override(observed,measurement,stamp,error,candidate,policy,clearance):
    if observed.get('reason')!='contact_band_unobserved':return observed
    if not measurement or measurement.get('depth_measured') is not True or measurement.get('stamp_ns')!=stamp:return observed
    delta=np.asarray(error,float)
    if delta.shape!=(3,) or not np.isfinite(delta).all() or np.any(abs(delta)>[.009,.006,.006]):
        return dict(observed,reason='current_target_not_aligned')
    width=candidate.get('observed_width_m',float('nan'))
    if not np.isfinite(width) or not 0<width<=policy.jaw_inner_width_m-2*policy.side_margin_m:
        return dict(observed,reason='material_outside_actual_open_gap')
    if not np.isfinite(clearance) or clearance<policy.table_margin_m-1e-6:
        return dict(observed,reason='actual_closing_hand_too_low',clearance_m=float(clearance))
    return dict(fits=True,reason='session6_contact_band_depth_check_disabled',
                trial_close=True,grasp_verified=False,current_contact_band_observed=False,
                current_target_depth_measured=True,clearance_m=float(clearance),
                alignment_error_m=delta.tolist(),registration_stamp_ns=stamp)

"""Can-only robust tabletop fallback; the verified apple estimator is unchanged.

Thin objects near table height must not bias the table plane. Fit its dominant
height band first and then select actual plane inliers. Original quality limits
remain in force; discarded points are NOT filled in the table support raster.
"""
import numpy as np

from apple_transfer_auto_plan import depth_in_map
from measure_destination_surface import measure as legacy_measure


def measure(depth,snapshot,destination):
    try:return legacy_measure(depth,snapshot,destination)
    except ValueError as exc:
        if str(exc)!='noisy_table_plane':raise
    mapped=depth_in_map(depth,snapshot);tf=snapshot['tracking_transforms']['map_from_base_at_depth']
    b=destination['axis_aligned_bounds'];center=np.array([b['center']['x'],b['center']['y']]);half=np.array([b['size']['x'],b['size']['y']])/2
    candidates=mapped[(abs(mapped[:,:2]-center)<half-.025).all(1)&(mapped[:,2]>.25)&(mapped[:,2]<1.05)]
    bins=np.floor(candidates[:,2]/.005).astype(int);keys,counts=np.unique(bins,return_counts=True);peak=keys[np.argmax(counts)]
    wide=candidates[abs(candidates[:,2]-(peak+.5)*.005)<.012]
    table=candidates[bins==peak]
    for _ in range(4):
        if len(table)<150:raise ValueError('insufficient_dominant_table_plane')
        design=np.c_[table[:,:2],np.ones(len(table))];coef,_,rank,_=np.linalg.lstsq(design,table[:,2],rcond=None)
        if rank!=3 or not np.isfinite(coef).all():raise ValueError('invalid_dominant_table_plane')
        residual=abs(np.c_[wide[:,:2],np.ones(len(wide))]@coef-wide[:,2]);table=wide[residual<.003]
    if (len(table)<150 or len(table)<.60*len(wide) or np.linalg.norm(coef[:2])>.10
            or min(np.ptp(table[:,:2],axis=0))<.08):raise ValueError('dominant_table_plane_not_established')
    coef=np.linalg.lstsq(np.c_[table[:,:2],np.ones(len(table))],table[:,2],rcond=None)[0]
    residual=abs(np.c_[table[:,:2],np.ones(len(table))]@coef-table[:,2]);p95=float(np.quantile(residual,.95))
    if p95>.004:raise ValueError('noisy_table_plane')
    return dict(destination_id=destination['id'],room=destination['room'],table_height_map_m=float(np.r_[center,1.]@coef),
        plane_map=coef.tolist(),plane_points=len(table),residual_p95_m=p95,
        observed_xy_min=table[:,:2].min(0).tolist(),observed_xy_max=table[:,:2].max(0).tolist(),
        footprint_center=center.tolist(),footprint_half_size=half.tolist(),base_map=tf,source_depth_stamp_ns=tf['stamp_ns'],
        estimator='dominant_height_seed_then_actual_depth_plane_inliers',near_height_outliers_removed=len(wide)-len(table),
        actual_plane_inlier_fraction=float(len(table)/len(wide)),residual_limit_relaxed=False,motion_authorized=False)

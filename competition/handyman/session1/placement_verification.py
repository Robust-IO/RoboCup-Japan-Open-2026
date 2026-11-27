"""Class-independent support checks; no robot actions or protocol publication.

Before release points may be current-TF held estimates. After release points must
be independently segmented RGBD observations of the target, never those estimates.
"""
import numpy as np


def support_check(points_map,plane,plan,support,policy,*,after_release=False):
    points=np.asarray(points_map,float);coef=np.asarray(plane['plane_map'],float)
    if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all() or coef.shape!=(3,) or not np.isfinite(coef).all():
        raise ValueError('finite_target_surface_required')
    if plane['destination_id']!=plan['destination_id']:
        raise ValueError('wrong_destination_plane')
    center=(points.min(0)+points.max(0))/2
    tight=plan.get("placement_selection_basis")=="tight_object_disk"
    if tight:
        from object_support_disk import enclosing_disk
        center[:2],_=enclosing_disk(points[:,:2])
    # A newly observed shelf layer need not match the approach planning plane.
    # Evaluate release geometry against the current measurement below.
    radius=float(np.linalg.norm(points[:,:2]-center[:2],axis=1).max())
    # Retain the measured pre-release footprint, preventing an occluded partial
    # observation from shrinking the accepted object footprint after release.
    radius=max(radius,float(plan['radius_m']))
    edge=support.query(center[:2],radius)
    gap=float(np.min(points[:,2]-np.c_[points[:,:2],np.ones(len(points))]@coef))
    center_based=plan.get('placement_selection_basis')=='observed_center_local_patch'
    if after_release or center_based:
        table_center=np.asarray(plan['footprint_center'],float)
        table_half=np.asarray(plan['footprint_half_size'],float)
        if table_center.shape!=(2,) or table_half.shape!=(2,) or not np.isfinite(np.r_[table_center,table_half]).all() or np.any(table_half<=0):
            raise ValueError('valid_destination_bounds_required')
        supported=bool((abs(center[:2]-table_center)<=table_half).all())
    else:supported=edge['safe']
    if tight:
        from object_support_disk import disk_inside
        edge=disk_inside(center[:2],radius,plan['support_footprint'],plan.get('object_disk_edge_gap_m',.005))
        supported=edge['safe']
    # Footprint containment is a planning/pre-release condition. After release,
    # current target/table observations are judged by height and stability only.
    if after_release:supported=True
    height_ok=abs(gap)<=policy.PLACED_BOTTOM_GAP_M if after_release else policy.RELEASE_BOTTOM_GAP_M[0]<=gap<=policy.RELEASE_BOTTOM_GAP_M[1]
    return dict(ready=bool(supported and height_ok),center_map_m=center.tolist(),
                bottom_gap_m=gap,edge=edge,edge_is_diagnostic_only=bool(after_release or (center_based and not tight)),
                acceptance_basis='destination_height_and_temporal_stability' if after_release else ('destination_center_and_height' if center_based else 'observed_support'),
                planned_xy_is_success_gate=False)


def verify_placement(rows,plan,support,policy,*,target,release_stamp_ns,joints,
                     required_frames=3,minimum_frame_spacing_ns=150_000_000):
    if required_frames<2 or len(rows)<required_frames or not policy.hand_is_open(joints[5]):
        raise ValueError('independent_frames_and_open_hand_required')
    checks=[];previous=release_stamp_ns
    for row in rows:
        stamp=row['depth_stamp_ns']
        if (stamp<=release_stamp_ns or stamp<=previous or
                (checks and stamp-previous<minimum_frame_spacing_ns)):
            raise ValueError('distinct_post_release_frames_required')
        if (row.get('target')!=target or row.get('destination_id')!=plan['destination_id']
                or row.get('geometry_source')!='independent_target_RGBD'
                or row.get('bottom_observed') is not True or not row.get('evidence_ref')
                or row['plane']['source_depth_stamp_ns']!=stamp):
            raise ValueError('independent_target_and_table_evidence_required')
        check=support_check(row['points_map'],row['plane'],plan,support,policy,after_release=True)
        if not check['ready']:raise ValueError('target_not_supported_on_destination')
        checks.append(check);previous=stamp
    centers=np.asarray([c['center_map_m'] for c in checks])
    spread=float(np.linalg.norm(centers-centers.mean(0),axis=1).max())
    if spread>policy.PLACED_CENTER_SPREAD_M:raise ValueError('placed_target_not_stable')
    return dict(status='ok',target=target,destination=plan['destination_id'],placement_verified=True,
                evidence_refs=[r['evidence_ref'] for r in rows],checks=checks,center_spread_m=spread,
                planned_xy_is_success_gate=False)

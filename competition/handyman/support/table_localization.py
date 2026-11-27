"""Pure scan alignment policy for stopped table operations; no ROS writes."""
import json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from hand_camera_ray import rotation
from match_reviewed_room_scan import search

def config():return json.loads(Path(__file__).with_suffix('.json').read_text())
def yaw(q):
    r=rotation(q);return math.atan2(r[1,0],r[0,0])
def angle(a):return math.atan2(math.sin(a),math.cos(a))
def pose(tf):return [*tf['translation'][:2],yaw(tf['quaternion'])]
def anchor_from_pose(matched,odom):
    r=Rotation.from_euler('z',matched[2]).as_matrix()@rotation(odom['quaternion']).T
    t=np.r_[matched[:2],0.]-r@np.asarray(odom['translation'])
    return dict(translation=t.tolist(),quaternion=Rotation.from_matrix(r).as_quat().tolist())
def compose(anchor,odom):
    r=rotation(anchor['quaternion']);t=r@np.asarray(odom['translation'])+anchor['translation']
    return dict(translation=t.tolist(),quaternion=Rotation.from_matrix(r@rotation(odom['quaternion'])).as_quat().tolist())
def stationary(before,after,cfg):
    return bool(np.linalg.norm(np.asarray(before['translation'])[:2]-np.asarray(after['translation'])[:2])<=cfg['maximum_stationary_translation_m']
        and abs(angle(yaw(before['quaternion'])-yaw(after['quaternion'])))<=cfg['maximum_stationary_yaw_rad'])
def align(row,grid,origin,resolution,cfg):
    ranges=np.array([np.nan if v is None else v for v in row['ranges']]);idx=np.flatnonzero(np.isfinite(ranges)&(ranges>.15)&(ranges<9))[::3]
    if len(idx)<80:raise ValueError('insufficient_actual_laser_points')
    a=row['angle_min']+idx*row['angle_increment'];points=np.c_[ranges[idx]*np.cos(a),ranges[idx]*np.sin(a),np.zeros(len(idx))]
    tf=row['base_from_scan'];points=(points@rotation(tf['quaternion']).T+tf['translation'])[:,:2];current=pose(row['map_from_base'])
    original=search(points,grid,origin,resolution,[(v,v) for v in current],[1,1,1])[0]
    radius=cfg['search_radius_m'];turn=cfg['search_yaw_rad']
    coarse=search(points,grid,origin,resolution,[(current[0]-radius,current[0]+radius),(current[1]-radius,current[1]+radius),(current[2]-turn,current[2]+turn)],[.05,.05,.025]);best=coarse[0][1]
    fine=search(points,grid,origin,resolution,[(best[0]-.06,best[0]+.06),(best[1]-.06,best[1]+.06),(best[2]-.025,best[2]+.025)],[.005,.005,.0025])[0]
    return dict(stamp_ns=row['stamp_ns'],current_pose=current,current_score=original[0],current_inlier_fraction=original[2],
        matched_pose=fine[1],matched_score=fine[0],matched_inlier_fraction=fine[2],odom_from_base=row['odom_from_base'],
        anchor=anchor_from_pose(fine[1],row['odom_from_base']))
def decide(results,cfg):
    if len(results)!=cfg['scans'] or len({x['stamp_ns'] for x in results})!=len(results):raise ValueError('distinct_laser_scans_required')
    for r in results:
        if not np.isfinite(np.r_[r['matched_pose'],r['matched_score'],r['matched_inlier_fraction']]).all():raise ValueError('invalid_laser_match')
        if r['matched_score']>cfg['maximum_score_m'] or r['matched_inlier_fraction']<cfg['minimum_inlier_fraction']:raise ValueError('laser_map_alignment_not_reliable')
        if np.linalg.norm(np.asarray(r['matched_pose'])[:2]-r['current_pose'][:2])>cfg['search_radius_m']*1.4:raise ValueError('match_beyond_local_search')
    # Compare the independently derived anchors, rather than requiring robot
    # poses from slightly different scan times to be numerically identical.
    anchors=np.array([r['anchor']['translation'][:2] for r in results]);median=np.median(anchors,axis=0);angles=np.array([yaw(r['anchor']['quaternion']) for r in results])
    mean_angle=float(np.arctan2(np.sin(angles).mean(),np.cos(angles).mean()))
    if max(np.linalg.norm(anchors-median,axis=1))>cfg['consensus_translation_m'] or max(abs(angle(v-mean_angle)) for v in angles)>cfg['consensus_yaw_rad']:
        raise ValueError('independent_laser_matches_disagree')
    best=min(results,key=lambda r:r['matched_score']);delta=np.asarray(best['matched_pose'])-best['current_pose']
    needs=bool(np.linalg.norm(delta[:2])>cfg['correction_translation_m'] or abs(angle(delta[2]))>cfg['correction_yaw_rad'])
    current_good=all(r['current_inlier_fraction']>=cfg['minimum_inlier_fraction'] and r['current_score']<=cfg['maximum_score_m'] for r in results)
    return dict(status='correction_required' if needs or not current_good else 'already_aligned',apply_correction=needs or not current_good,
        anchor=best['anchor'],matched_pose=best['matched_pose'],correction_xy_yaw=delta.tolist(),scans=results)

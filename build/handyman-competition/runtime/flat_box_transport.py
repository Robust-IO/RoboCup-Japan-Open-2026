"""Box held geometry is kinematic; silhouette checks use current hand RGB."""
import numpy as np
import cv2
from horizontal_can_transport_geometry import palm_pose as legacy_palm_pose
import json
from pathlib import Path
from hand_camera_ray import rotation

def palm_pose(snapshot):
    if 'base_from_palm_at_depth' in snapshot:
        tf=snapshot['base_from_palm_at_depth'];return np.asarray(tf['translation']),rotation(tf['quaternion'])
    return legacy_palm_pose(snapshot)

def from_grasp(grasp):
    grasp=Path(grasp);prior=json.loads((grasp/'result.json').read_text())
    if not prior.get('grasp_success_verified') or prior.get('release_commands_sent')!=0:raise ValueError('actual_verified_unreleased_box_required')
    held=json.loads((grasp/'held-context.json').read_text());model=held['model']
    source_path=grasp/'source-snapshot.json'
    source=json.loads((source_path if source_path.exists() else grasp/'captures/source/snapshot.json').read_text())
    rows=prior.get('lift_evidence',[])
    proof=Path(rows[-1]['capture']) if rows else grasp/'captures/lift-proof-2'
    if not proof.resolve().is_relative_to(grasp.resolve()):raise ValueError('lift_proof_outside_grasp_run')
    last=json.loads((proof/'snapshot.json').read_text())
    a=np.asarray(model['axis_base']);axes=np.column_stack((a,[-a[1],a[0],0.],[0.,0.,1.]))
    ro=rotation(source['tracking_transforms']['odom_from_base_at_depth']['quaternion']);rn=rotation(last['tracking_transforms']['odom_from_base_at_depth']['quaternion']);_,rp=palm_pose(last)
    held.update(axes_palm=(rp.T@rn.T@ro@axes).tolist(),sizes_m=[model['length_m'],model['width_m'],model['height_m']],reference_hand_capture=str(proof))
    return held,prior,cv2.imread(str(proof/'hand.png'))

def extents(axes,sizes):
    axes=np.asarray(axes,float);sizes=np.asarray(sizes,float)
    if axes.shape!=(3,3) or sizes.shape!=(3,) or not np.isfinite([*axes.ravel(),*sizes]).all() or (sizes<=0).any():
        raise ValueError('invalid_box_extents')
    if not np.allclose(axes.T@axes,np.eye(3),atol=.002):raise ValueError('orthogonal_box_axes_required')
    corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*sizes/2
    rotated=corners@axes.T
    return float(abs(rotated[:,2]).max()),float(np.linalg.norm(rotated[:,:2],axis=1).max())

def shape(snapshot,held):
    p,r=palm_pose(snapshot);center=p+r@held['center_palm_m'];axes=r@np.asarray(held['axes_palm'])
    vertical,horizontal=extents(axes,held['sizes_m'])
    return dict(class_name='cigarette',shape='flat_cuboid',center_m=center.tolist(),axes_base=axes.tolist(),
        height_m=held['sizes_m'][2],sizes_m=held['sizes_m'],horizontal_support_radius_m=horizontal,
        bottom_vertical_extent_m=vertical,estimated_lowest_point_base_m=(center-[0,0,vertical]).tolist(),
        stamp_ns=snapshot['streams']['depth']['stamp_ns'],object_depth_measured=False,
        position_is_kinematic_estimate=True,source='verified_box_grasp_rigid_pose_with_current_hand_RGB')

def silhouette(reference,current,appearance):
    def components(image):
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);d=abs(hsv[:,:,0].astype(float)-appearance['hue']);d=np.minimum(d,180-d)
        mask=((d<=appearance['tolerance'])&(hsv[:,:,1]>70)&(hsv[:,:,2]>30)).astype(np.uint8)
        return cv2.connectedComponentsWithStats(cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8)),8)
    count,labels,stats,centers=components(reference)
    candidates=sorted(range(1,count),key=lambda i:stats[i,cv2.CC_STAT_AREA],reverse=True)
    if not candidates or stats[candidates[0],cv2.CC_STAT_AREA]<100:raise ValueError('verified_box_hand_reference_missing')
    best=candidates[0]
    if len(candidates)>1 and stats[best,cv2.CC_STAT_AREA]<5*stats[candidates[1],cv2.CC_STAT_AREA]:raise ValueError('ambiguous_box_hand_reference')
    old=labels==best;old_center=centers[best]
    count,labels,stats,centers=components(current)
    associated=[i for i in range(1,count) if np.count_nonzero((labels==i)&old)>.5*old.sum()]
    if len(associated)!=1:raise ValueError('same_held_box_not_seen_in_hand_region')
    best=associated[0];new=labels==best;ratio=float(new.sum()/old.sum())
    overlap=float(np.count_nonzero(old&new)/np.count_nonzero(old|new));shift=float(np.linalg.norm(centers[best]-old_center))
    if not .75<=ratio<=1.30 or overlap<.70 or shift>20:raise ValueError('held_box_changed_relative_to_hand')
    return dict(evidence_type='current_same_flat_box_silhouette_at_verified_grip',iou=overlap,area_ratio=ratio,
        center_shift_pixels=shift,source_pixels=int(old.sum()),current_pixels=int(new.sum()),object_depth_measured=False)

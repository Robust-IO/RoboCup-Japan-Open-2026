"""Observed tabletop/front-face edge for the current source-side base check.

Furniture identity stays in the imported map. Only the approached face of its
coarse footprint is corrected from connected tabletop and visible front face.
No hidden table boundary is inferred from an image crop or a depth hole.
"""
import copy
import cv2,numpy as np
from held_can_appearance import pixel_points
from hand_camera_ray import rotation
from apple_transfer_auto_plan import footprint,map_pose

def observed_front(depth,snapshot,table,plane):
    if plane['source_depth_stamp_ns']!=snapshot['streams']['depth']['stamp_ns']:raise ValueError('same_frame_source_edge_required')
    points=pixel_points(depth,snapshot);tf=snapshot['tracking_transforms']['map_from_base_at_depth'];mapped=points@rotation(tf['quaternion']).T+tf['translation']
    coef=np.asarray(plane['plane_map']);residual=mapped[:,:,2]-(mapped[:,:,:2]@coef[:2]+coef[2]);c,h=footprint(table)
    valid=np.isfinite(depth)&(depth>0);mask=valid&(abs(residual)<.004)
    count,labels,_,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
    seed=mask&(abs(mapped[:,:,:2]-c)<h).all(2)
    counts=np.bincount(labels[seed],minlength=count);counts[0]=0
    if not counts.any():raise ValueError('source_tabletop_component_not_seen')
    component=labels==int(counts.argmax());edge=np.zeros_like(mask)
    # The image must show top followed by the lower vertical front face,
    # not just background/floor, image boundary, a depth hole, or an occluder.
    edge[:-4]=component[:-4]&~component[4:]&valid[4:]&(residual[4:]<-.008)&(residual[4:]>-.10)
    candidates=mapped[edge]
    if len(candidates)<30:raise ValueError('actual_source_front_face_edge_not_seen')
    start=np.asarray(map_pose(snapshot));side=int(np.argmax(abs((start[:2]-c)/h)));sign=-1 if start[side]<c[side] else 1
    q=float(np.quantile(sign*candidates[:,side],.85));face=sign*q
    actual=copy.deepcopy(table);center=c.copy();half=h.copy();low=c-h;high=c+h
    # A 1cm allowance keeps the corrected boundary on the robot's side
    # of the observed edge. Only the current approach face is corrected.
    if sign<0:low[side]=face-.01
    else:high[side]=face+.01
    if low[side]>=high[side]:raise ValueError('observed_front_does_not_bound_source_table')
    center=(low+high)/2;half=(high-low)/2
    actual['axis_aligned_bounds']=dict(center=dict(x=float(center[0]),y=float(center[1])),size=dict(x=float(2*half[0]),y=float(2*half[1])))
    proof=dict(source='current_connected_top_and_lower_front_face_depth',source_depth_stamp_ns=plane['source_depth_stamp_ns'],edge_points=len(candidates),
        approached_axis=side,approached_sign=sign,observed_front_map_m=face,original_front_map_m=float((c+sign*h)[side]),
        corrected_front_map_m=float((low if sign<0 else high)[side]),semantic_identity_unchanged=True)
    return actual,proof

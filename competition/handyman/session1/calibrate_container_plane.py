"""Intersect reviewed inner-mouth rays with a measured sloping lip plane."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def calibrate(capture,mouth_pixels,lip_pixels,destination_id):
    root=Path(capture);s=json.loads((root/'snapshot.json').read_text())
    if abs(s['streams']['rgb']['stamp_ns']-s['streams']['depth']['stamp_ns'])>30000000:raise ValueError('unsynchronized_capture')
    if (root/'points_map.npy').exists():a=np.load(root/'points_map.npy')
    else:
        depth=np.load(root/'depth_m.npy');v,u=np.indices(depth.shape)
        k=np.asarray(s['rgb_k']).reshape(3,3)
        a=np.stack([(u-k[0,2])*depth/k[0,0],(v-k[1,2])*depth/k[1,1],depth],axis=-1)
        a[~np.isfinite(depth)|(depth<=.05)]=np.nan
        for tf in (s['base_from_rgb'],s['tracking_transforms']['map_from_base_at_depth']):
            a=a@Rotation.from_quat(tf['quaternion']).as_matrix().T+tf['translation']
    samples=[]
    for x,y in lip_pixels:
        q=a[y-1:y+2,x-1:x+2].reshape(-1,3);samples.extend(q[np.isfinite(q).all(1)])
    pts=np.asarray(samples);center=pts.mean(0)
    _,sv,vh=np.linalg.svd(pts-center);normal=vh[-1]
    residual=float(np.max(abs((pts-center)@normal)))
    if sv[1]<.01 or residual>.005:raise ValueError('lip_plane_not_supported')
    t=s['tracking_transforms']['map_from_base_at_depth'];b=s['base_from_rgb']
    r=Rotation.from_quat(t['quaternion']).as_matrix();origin=r@b['translation']+t['translation']
    r=r@Rotation.from_quat(b['quaternion']).as_matrix();k=np.asarray(s['rgb_k']).reshape(3,3)
    corners=[]
    for x,y in mouth_pixels:
        ray=r@np.linalg.solve(k,[x,y,1]);den=float(ray@normal)
        if abs(den)<1e-6:raise ValueError('parallel_mouth_ray')
        distance=float((center-origin)@normal)/den
        if distance<=0:raise ValueError('mouth_behind_camera')
        corners.append(origin+distance*ray)
    corners=np.asarray(corners)
    return dict(destination_id=destination_id,polygon_map_m=corners[:,:2].tolist(),
        rim_height_m=float(pts[:,2].max()),source='reviewed_inner_edge_rays_intersect_measured_sloping_lip',
        reviewed_pixels=mouth_pixels,reviewed_rim_pixels=lip_pixels,measured_corners_map_m=corners.tolist(),
        lip_plane_max_residual_m=residual,evidence_ref=str(root),depth_stamp_ns=s['streams']['depth']['stamp_ns'],
        autonomous_opening_detection=False,object_truth_coordinates_used=False)

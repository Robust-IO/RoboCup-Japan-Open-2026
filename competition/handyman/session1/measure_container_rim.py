"""Colour-independent RGBD measurement using reviewed mouth and highest-rim pixels."""
import json
from pathlib import Path
import numpy as np

def measure(capture,mouth_pixels,rim_pixels,destination_id):
    from hand_camera_ray import rotation
    root=Path(capture);s=json.loads((root/'snapshot.json').read_text());depth=np.load(root/'depth_m.npy');k=np.asarray(s['rgb_k']).reshape(3,3)
    if abs(s['streams']['rgb']['stamp_ns']-s['streams']['depth']['stamp_ns'])>30_000_000:raise ValueError('synchronized_capture_required')
    def point(pixel):
        x,y=map(int,pixel)
        if not 0<=y<depth.shape[0] or not 0<=x<depth.shape[1]:raise ValueError('reviewed_pixel_outside_image')
        z=float(depth[y,x])
        if not np.isfinite(z) or z<=.05:raise ValueError('reviewed_pixel_has_no_depth')
        p=np.array([(x-k[0,2])*z/k[0,0],(y-k[1,2])*z/k[1,1],z])
        for tf in (s['base_from_rgb'],s['tracking_transforms']['map_from_base_at_depth']):p=p@rotation(tf['quaternion']).T+tf['translation']
        return p
    corners=np.array([point(p) for p in mouth_pixels]);rim=np.array([point(p) for p in rim_pixels])
    if len(corners)<3 or len(rim)<3:raise ValueError('reviewed_mouth_and_rim_samples_required')
    result=dict(destination_id=destination_id,polygon_map_m=corners[:,:2].tolist(),rim_height_m=float(rim[:,2].max()),
        source='reviewed_inner_mouth_and_highest_rim_rgbd_pixels',reviewed_pixels=mouth_pixels,reviewed_rim_pixels=rim_pixels,
        measured_corners_map_m=corners.tolist(),measured_rim_map_m=rim.tolist(),evidence_ref=str(root),depth_stamp_ns=s['streams']['depth']['stamp_ns'],
        autonomous_opening_detection=False,object_truth_coordinates_used=False)
    from container_transfer import release_pose
    release_pose(result,0.)
    return result

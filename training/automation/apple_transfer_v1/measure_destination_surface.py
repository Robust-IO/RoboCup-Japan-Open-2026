"""Read-only depth-plane measurement within an exported furniture footprint."""
import argparse
import json
from pathlib import Path
import numpy as np
from hand_camera_ray import rotation


def measure(depth,snapshot,destination):
    tf=snapshot['tracking_transforms']['map_from_base_at_depth']
    if tf['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']:raise ValueError('map_transform_not_image_time')
    v,u=np.indices(depth.shape);good=np.isfinite(depth)&(depth>.05)&(depth<4)
    k=np.array(snapshot['rgb_k']).reshape(3,3);z=depth[good];t=snapshot['base_from_rgb']
    points=np.c_[(u[good]-k[0,2])*z/k[0,0],(v[good]-k[1,2])*z/k[1,1],z]@rotation(t['quaternion']).T+t['translation']
    mapped=points@rotation(tf['quaternion']).T+tf['translation']
    bounds=destination['axis_aligned_bounds'];center=np.array([bounds['center']['x'],bounds['center']['y']]);half=np.array([bounds['size']['x'],bounds['size']['y']])/2
    mask=(np.abs(mapped[:,:2]-center)<half-.025).all(axis=1)&(mapped[:,2]>.25)&(mapped[:,2]<1.05)
    candidates=mapped[mask]
    if len(candidates)<150:raise ValueError('insufficient_table_depth')
    bins=np.floor(candidates[:,2]/.005).astype(int);keys,counts=np.unique(bins,return_counts=True);peak=keys[np.argmax(counts)]
    table=candidates[np.abs(candidates[:,2]-(peak+.5)*.005)<.012]
    for _ in range(3):
        design=np.c_[table[:,:2],np.ones(len(table))];coef=np.linalg.lstsq(design,table[:,2],rcond=None)[0]
        residual=np.abs(design@coef-table[:,2]);table=table[residual<.006]
    if len(table)<150 or np.linalg.norm(coef[:2])>.10 or min(np.ptp(table[:,:2],axis=0))<.08:raise ValueError('invalid_table_plane')
    residual=np.abs(np.c_[table[:,:2],np.ones(len(table))]@coef-table[:,2])
    if np.quantile(residual,.95)>.004:raise ValueError('noisy_table_plane')
    return dict(destination_id=destination['id'],room=destination['room'],table_height_map_m=float(np.r_[center,1.]@coef),plane_map=coef.tolist(),plane_points=len(table),residual_p95_m=float(np.quantile(residual,.95)),observed_xy_min=table[:,:2].min(axis=0).tolist(),observed_xy_max=table[:,:2].max(axis=0).tolist(),footprint_center=center.tolist(),footprint_half_size=half.tolist(),base_map=tf,source_depth_stamp_ns=tf['stamp_ns'],motion_authorized=False)


def main():
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('--semantic-map',type=Path,required=True);p.add_argument('--destination-id',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('output_exists')
    dest=next(x for x in json.loads(a.semantic_map.read_text())['destinations'] if x['id']==a.destination_id)
    result=measure(np.load(a.capture/'depth_m.npy'),json.loads((a.capture/'snapshot.json').read_text()),dest)
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()

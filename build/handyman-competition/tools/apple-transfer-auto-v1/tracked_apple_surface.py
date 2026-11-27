"""Measure a previously verified held apple using palm-relative RGBD association.

No detector confidence is invented. This is not an open-world apple detector.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
from hand_camera_ray import rotation


def measure(rgb,depth,snapshot,expected):
    if abs(snapshot['streams']['rgb']['stamp_ns']-snapshot['streams']['depth']['stamp_ns'])>50_000_000:raise ValueError('rgb_depth_unsynchronised')
    k=np.asarray(snapshot['rgb_k']).reshape(3,3);v,u=np.indices(depth.shape);z=depth;tf=snapshot['base_from_rgb']
    points=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),-1)@rotation(tf['quaternion']).T+tf['translation']
    r,g,b=rgb.astype(float).transpose(2,0,1)
    valid=np.isfinite(z)&(z>0)&(z<3)&(r>80)&(r>1.35*g)&(r>1.2*b)&(b>.9*g)&(np.linalg.norm(points-expected,axis=2)<.09)
    pts=points[valid]
    if len(pts)<80:raise ValueError('insufficient_tracked_surface')
    origin=pts.mean(0);local=pts-origin;sv=np.linalg.svd(local,compute_uv=False)
    if sv[-1]/sv[0]<.035:raise ValueError('insufficient_curvature')
    q=np.linalg.lstsq(np.c_[2*local,np.ones(len(local))],(local*local).sum(1),rcond=None)[0]
    radius2=q[3]+q[:3]@q[:3]
    if not .015**2<=radius2<=.065**2:raise ValueError('invalid_radius')
    radius=float(np.sqrt(radius2));center=q[:3]+origin;residual=float(np.quantile(abs(np.linalg.norm(pts-center,axis=1)-radius),.95))
    if residual>.012 or np.linalg.norm(center-expected)>.025:raise ValueError('tracked_surface_not_consistent_with_palm')
    return dict(center_base=center.tolist(),radius_m=radius,residual_p95_m=residual,pixels=len(pts),association_error_m=float(np.linalg.norm(center-expected)),method='previously_verified_apple_palm_relative_red_rgbd_surface',detector_confidence=None,new_identity_verified=False,motion_authorized=False)


def main():
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('--expected-center',nargs=3,type=float,required=True);p.add_argument('--prior-verified-target',required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--release-map-xy',nargs=2,type=float);a=p.parse_args()
    if a.output.exists() or a.prior_verified_target!='apple-white-table-20261017':raise ValueError('verified_prior_track_required')
    s=json.loads((a.capture/'snapshot.json').read_text());result=measure(np.asarray(Image.open(a.capture/'rgb.png').convert('RGB')),np.load(a.capture/'depth_m.npy'),s,np.array(a.expected_center))
    tf=s['tracking_transforms']['map_from_base_at_depth'];result['center_map']=(rotation(tf['quaternion'])@result['center_base']+tf['translation']).tolist();result['source_stamp_ns']=s['streams']['depth']['stamp_ns'];result['prior_verified_target']=a.prior_verified_target
    from verify_apple_lift_sequence import palm_in_base
    palm=palm_in_base(s);result['palm_base']=palm.tolist();result['estimated_bottom_below_palm_m']=float(palm[2]-result['center_base'][2]+result['radius_m'])
    r=rotation(tf['quaternion']);result['base_map_pose']=[*tf['translation'][:2],float(np.arctan2(r[1,0],r[0,0]))]
    if a.release_map_xy is not None:
        if not np.isfinite(a.release_map_xy).all():raise ValueError('invalid_release_xy')
        delta_map=np.r_[np.asarray(a.release_map_xy)-result['center_map'][:2],0.]
        result['planned_base_delta_to_release_xy_m']=(r.T@delta_map)[:2].tolist()
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()

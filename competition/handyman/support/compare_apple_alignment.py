"""Session-specific read-only apple surface evidence; never a grasp command."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from hand_camera_ray import rotation, rays


def surface(folder, box, red_only=False):
    s=json.loads((folder/'snapshot.json').read_text());d=np.load(folder/'depth_m.npy')
    image=cv2.imread(str(folder/'rgb.png'));k=np.array(s['rgb_k']).reshape(3,3)
    v,u=np.indices(d.shape);t=s['base_from_rgb']
    xyz=np.stack([(u-k[0,2])*d/k[0,0],(v-k[1,2])*d/k[1,1],d],-1)@rotation(t['quaternion']).T+t['translation']
    x1,y1,x2,y2=box
    valid=(u>=x1)&(u<x2)&(v>=y1)&(v<y2)&np.isfinite(d)&(d>0)
    if red_only:
        b,g,r=cv2.split(image.astype(float));valid &= (r>80)&(r>g*1.35)&(r>b*1.2)
    if valid.sum()<20:raise ValueError('Insufficient surface evidence')
    return dict(samples=int(valid.sum()),median_base=np.median(xyz[valid],axis=0).tolist(),
                depth_quantiles=np.quantile(d[valid],[.05,.5,.95]).tolist(),
                box=box,red_only=red_only,depth_stamp_ns=s['streams']['depth']['stamp_ns'])


def main():
    p=argparse.ArgumentParser();p.add_argument('before',type=Path);p.add_argument('after',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    before=surface(a.before,[311,211,331,232])
    # Manually reviewed exposed red apple pixels in the occluded head frame;
    # color is used only for this diagnostic, not as a production classifier.
    after=surface(a.after,[301,302,337,334],True)
    s=json.loads((a.after/'snapshot.json').read_text())
    ray=rays(s,[142.25453186,251.82557678,201.13162231,298.86068726],'sigverse_hsr_hand_v2')['base_ray']
    target=np.array(before['median_base']);origin=np.array(ray['origin']);direction=np.array(ray['unit_direction'])
    distance=float(np.linalg.norm(np.cross(target-origin,direction)))
    # Palm center from this trial TF, finger entrance center offset from audit.
    provisional_contact=np.array([.627169345+.06212587,.078,.72])
    report=dict(motion_authorized=False,before=before,after=after,hand_ray=ray,
        before_surface_to_hand_ray_distance_m=distance,
        provisional_surface_minus_contact_m=(target-provisional_contact).tolist(),
        limitations=['Visible surface is not object center; no final grasp depth established.',
        'Frames acquired at different times; base assumed stationary from completed trial, not tracked here.',
        'Partially visible red ROI diagnostic is manually selected; not production segmentation.',
        'No motion authorized by these offsets. Fresh localization and swept collision check required before motion.'])
    with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()

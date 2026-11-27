"""Manually reviewed scene diagnostic, no control or automatic object identity."""
import argparse
import json
from pathlib import Path
import numpy as np
from compare_apple_alignment import surface
from hand_camera_ray import rotation


def main():
    p=argparse.ArgumentParser()
    p.add_argument('capture',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();s=json.loads((a.capture/'snapshot.json').read_text())
    if abs(s['joints']['head_tilt_joint']+.8)>.005:raise ValueError('ROI requires reviewed low head pose')
    apple=surface(a.capture,[245,25,330,125],True)
    d=np.load(a.capture/'depth_m.npy');v,u=np.indices(d.shape)
    k=np.array(s['rgb_k']).reshape(3,3);t=s['base_from_rgb']
    xyz=np.stack([(u-k[0,2])*d/k[0,0],(v-k[1,2])*d/k[1,1],d],-1)@rotation(t['quaternion']).T+t['translation']
    mask=(u>=310)&(u<=590)&(v>=180)&(v<=400)&(xyz[:,:,2]>.448)&(xyz[:,:,2]<.485)&(xyz[:,:,0]>.35)&(xyz[:,:,0]<.7)&np.isfinite(d)&(d>0)
    pts=xyz[mask,:2]
    if len(pts)<500:raise ValueError('Insufficient table fascia evidence')
    center=np.median(pts,axis=0);_,_,vh=np.linalg.svd(pts-center,full_matrices=False)
    n=vh[-1];n=n if n[0]>0 else -n
    offset=float(np.median(pts@n));residual=float(np.quantile(abs(pts@n-offset),.95))
    if residual>.005 or n[0]<.9:raise ValueError('Table plane does not match reviewed scene')
    target=np.array(apple['median_base']);contact=np.array([.689295215,.078])
    delta=target[:2]-contact
    # A staging proposal leaves 5cm between observed surface and provisional entrance.
    stage=delta-n*.05
    report=dict(motion_authorized=False,capture=str(a.capture),apple_visible_surface=apple,
        table_front_plane=dict(normal_xy=n.tolist(),offset_m=offset,samples=len(pts),p95_residual_m=residual),
        surface_to_edge_horizontal_m=float(target[:2]@n-offset),
        provisional_surface_alignment_translation_m=delta.tolist(),
        observation_standoff_translation_m=stage.tolist(),
        current_edge_x_at_hand_y_m=float((offset-n[1]*.078)/n[0]),
        limitations=['Manual red ROI is a diagnostic, not a production detector.',
        'Visible surface is not target center; alignment does not guarantee enclosure.',
        'Table front fitted to selected visible fascia, not all furniture geometry.',
        'No collision/path authorization; screen lifted arm before further base approach.'])
    with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()

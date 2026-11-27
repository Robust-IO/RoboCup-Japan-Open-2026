"""Offline RGBD furniture measurement from explicitly reviewed inner-rim pixels.

The reviewed pixels describe the BIN, not a target object's grasp coordinate.
Output includes source image/stamp and measured corners for later audit.
"""
import argparse,json
from pathlib import Path
import numpy as np

def measure(capture,vertices,destination_id):
    from hand_camera_ray import rotation
    import cv2
    capture=Path(capture);s=json.loads((capture/'snapshot.json').read_text());d=np.load(capture/'depth_m.npy')
    rgb=cv2.imread(str(capture/'rgb.png'));k=np.asarray(s['rgb_k']).reshape(3,3)
    if abs(s['streams']['rgb']['stamp_ns']-s['streams']['depth']['stamp_ns'])>30_000_000:raise ValueError('synchronized_capture_required')
    def mapped(x,y,z):
        p=np.array([(x-k[0,2])*z/k[0,0],(y-k[1,2])*z/k[1,1],z])
        t=s['base_from_rgb'];p=p@rotation(t['quaternion']).T+t['translation']
        t=s['tracking_transforms']['map_from_base_at_depth'];return p@rotation(t['quaternion']).T+t['translation']
    points=[]
    for x,y in vertices:
        z=float(d[y,x])
        if not np.isfinite(z) or z<=.05:raise ValueError('rim_pixel_depth_missing')
        points.append(mapped(x,y,z).tolist())
    # Highest observed blue lip in a narrow image region around the reviewed
    # opening. The opening itself is sloped; do not treat its center depth as rim.
    x0,y0=np.min(vertices,axis=0)-30;x1,y1=np.max(vertices,axis=0)+30
    hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV);v,u=np.indices(d.shape)
    mask=(u>=x0)&(u<=x1)&(v>=y0)&(v<=y1)&(hsv[:,:,0]>90)&(hsv[:,:,0]<120)&(hsv[:,:,1]>80)&np.isfinite(d)&(d>.05)&(d<3)
    ys,xs=np.nonzero(mask)
    if not len(xs):raise ValueError('blue_rim_not_observed')
    z=np.asarray([mapped(int(x),int(y),float(d[y,x]))[2] for x,y in zip(xs,ys)])
    return dict(destination_id=destination_id,polygon_map_m=[p[:2] for p in points],rim_height_m=float(np.quantile(z,.99)),
                source='reviewed_inner_rim_pixels_and_current_rgbd',reviewed_pixels=[list(p) for p in vertices],
                measured_corners_map_m=points,evidence_ref=str(capture),depth_stamp_ns=s['streams']['depth']['stamp_ns'],
                autonomous_opening_detection=False,object_truth_coordinates_used=False)

def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--destination',required=True);p.add_argument('--rim-pixels',type=int,nargs=8,required=True);a=p.parse_args()
    result=measure(a.capture,list(zip(a.rim_pixels[::2],a.rim_pixels[1::2])),a.destination)
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()

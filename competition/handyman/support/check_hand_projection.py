"""Read-only live-TF finger projection comparison; not a collision certificate."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from hand_camera_ray import rotation, HAND_RASTER_TO_FRAME


def project(snapshot, geometry, corrected):
    size=snapshot['streams']['hand']
    mask=np.zeros((size['height'],size['width']),np.uint8)
    k=np.array(snapshot['hand_k']).reshape(3,3)
    raster_to_frame=HAND_RASTER_TO_FRAME if corrected else np.eye(3)
    for part in geometry['parts']:
        key='hand_from_'+part['link']
        if key not in snapshot:continue
        v=np.array([[p['x'],p['y'],p['z'],1.] for p in part['vertices']])
        v=(np.array(part['meshToLinkUnityRowMajor']).reshape(4,4)@v.T).T[:,:3]
        v[:,0]*=-1  # Unity link coordinates to ROS link coordinates.
        tf=snapshot[key]
        v=(rotation(tf['quaternion'])@v.T).T+tf['translation']
        v=(raster_to_frame.T@v.T).T
        triangles=np.array(part['triangles']).reshape(-1,3)
        for indices in triangles:
            triangle=v[indices]
            # Skip near-plane crossing triangles instead of unsafe division.
            if np.any(triangle[:,2]<=.005):continue
            uv=(k@triangle.T).T
            uv=uv[:,:2]/uv[:,2,None]
            if not np.isfinite(uv).all() or np.max(np.abs(uv))>1e6:continue
            cv2.fillConvexPoly(mask,np.rint(uv).astype(np.int32),255)
    return mask


def main():
    p=argparse.ArgumentParser();p.add_argument('snapshot',type=Path);p.add_argument('geometry',type=Path);p.add_argument('image',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    snapshot=json.loads(a.snapshot.read_text());geometry=json.loads(a.geometry.read_text())
    rgb=cv2.imread(str(a.image))
    if rgb is None:raise ValueError('Missing image')
    a.output.mkdir(parents=True)
    for label,corrected in [('uncorrected',False),('source_corrected',True)]:
        mask=project(snapshot,geometry,corrected)
        if mask.shape!=rgb.shape[:2]:raise ValueError('Image dimensions mismatch')
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        overlay=rgb.copy();cv2.drawContours(overlay,contours,-1,(255,255,0),1)
        cv2.imwrite(str(a.output/(label+'.png')),overlay)
        cv2.imwrite(str(a.output/(label+'-mask.png')),mask)
        print(label,'projected_pixels',np.count_nonzero(mask))
    (a.output/'scope.json').write_text(json.dumps(dict(motion_authorized=False,
        limitations=['Finger meshes only; near-plane crossing triangles skipped.',
                     'Visual contour comparison, no automatic pass or grasp authorization.',
                     'Single posture does not validate all wrist/arm configurations.']),indent=2))


if __name__=='__main__':main()

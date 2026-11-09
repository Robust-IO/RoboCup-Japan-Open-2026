"""Offline real-RGBD regression. Never imports a ROS controller or sends motion."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from pepper_geometry import estimate_source,measure_near
from canned_grasp_geometry import transform_point

def replay(reference):
    reference=Path(reference);manifest=json.loads((reference/'manifest.json').read_text());rows=[]
    for item in manifest['samples']:
        path=reference/'frames'/item['id']
        for name,digest in item['hashes'].items():
            if hashlib.sha256((path/name).read_bytes()).hexdigest()!=digest:raise ValueError('reference_frame_changed')
        rgb=cv2.cvtColor(cv2.imread(str(path/'rgb.png')),cv2.COLOR_BGR2RGB)
        depth=np.load(path/'depth_m.npy',allow_pickle=False);s=json.loads((path/'snapshot.json').read_text())
        g=estimate_source(rgb,depth,s,item['detection'])
        m=measure_near(rgb,depth,s,np.asarray(g['center_m'])+[.004,-.003,.002],
            np.asarray(g['cap_center_m'])+[.004,-.003,.002],g)
        error=float(np.linalg.norm(np.asarray(m['center_m'])-g['center_m']))
        if error>.003:raise ValueError('replayed_pepper_tracking_changed_center')
        rows.append(dict(id=item['id'],source_capture=item['capture'],source_geometry=g,current_surface=m,
            tracking_error_m=error,body_center_odom_m=transform_point(s,g['center_m']).tolist()))
    centers=np.asarray([r['body_center_odom_m'] for r in rows])
    return dict(status='offline_saved_pepper_rgbd_verified_not_physical_grasp',class_name='ground_pepper',
        frames=len(rows),robot_commands_sent=False,physical_grasp_verified=False,rows=rows,
        body_center_odom_spread_m=float(np.max(np.linalg.norm(centers-centers.mean(0),axis=1))),
        height_range_m=[min(r['source_geometry']['height_m'] for r in rows),max(r['source_geometry']['height_m'] for r in rows)],
        body_radius_range_m=[min(r['source_geometry']['body_radius_m'] for r in rows),max(r['source_geometry']['body_radius_m'] for r in rows)])

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():raise ValueError('output_exists')
    result=replay(args.reference);args.output.write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}))

if __name__=='__main__':main()

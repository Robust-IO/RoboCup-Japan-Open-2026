"""Saved RGBD fitter diagnostics, retaining the rejected original result."""
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image

CODE=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(CODE))
from object_surface_geometry import estimate,support_plane
import upright_cylinder_geometry as cylinder

BASE=Path('/__HANDYMAN_PACKAGE__/runtime')
CAP=BASE/'canned-source-observation-20261023-01/cameras/000002'
OUT=BASE/'canned-geometry-diagnostics-20261023-01';OUT.mkdir(exist_ok=False)
snapshot=json.loads((CAP/'snapshot.json').read_text())
report=json.loads((BASE/'canned-source-analysis-20261023-02/000002-analysis.json').read_text())
detections=[d for d in report['detections'] if d['name']=='canned_juice']
if len(detections)!=1:raise ValueError('unique_saved_target_required')
original=cylinder.cylinder_from_points
diagnostic={}
def examine(points,surrounding,profile):
    coef,plane_points,residual=support_plane(surrounding)
    heights=points[:,2]-np.c_[points[:,:2],np.ones(len(points))]@coef
    points=points[(heights>.004)&(heights<profile['height_range_m'][1]+.02)]
    heights=points[:,2]-np.c_[points[:,:2],np.ones(len(points))]@coef
    height=float(np.quantile(heights,.99));top=points[np.abs(heights-height)<.005]
    side=points[(heights>.15*height)&(heights<.8*height)]
    origin=side[:,:2].mean(axis=0);local=side[:,:2]-origin
    design=np.c_[2*local,np.ones(len(local))]
    q,_,rank,_=np.linalg.lstsq(design,np.sum(local*local,axis=1),rcond=None)
    squared=q[2]+q[:2]@q[:2]
    diagnostic.update(support_plane=coef.tolist(),support_residual_m=residual,object_height_m=height,
        point_count=len(points),top_points=len(top),side_points=len(side),side_svd=np.linalg.svd(local,compute_uv=False).tolist(),
        initial_algebraic_circle_radius_m=float(np.sqrt(squared)) if squared>0 else None,
        initial_circle_center_xy=(origin+q[:2]).tolist(),rank=int(rank),
        object_min=points.min(axis=0).tolist(),object_max=points.max(axis=0).tolist(),
        side_quantiles=np.quantile(side,[0,.01,.1,.5,.9,.99,1],axis=0).tolist())
    np.savez(OUT/'points.npz',points=points,surrounding=surrounding,side=side,top=top)
    return original(points,surrounding,profile)
cylinder.cylinder_from_points=examine
try:
    result=estimate(np.asarray(Image.open(CAP/'rgb.png').convert('RGB')),
        np.load(CAP/'depth_m.npy',allow_pickle=False),snapshot,detections[0],'canned_juice')
    diagnostic['geometry']=result
except ValueError as exc:diagnostic['rejected_reason']=str(exc)
(OUT/'result.json').write_text(json.dumps(diagnostic,indent=2));print(json.dumps(diagnostic),flush=True)

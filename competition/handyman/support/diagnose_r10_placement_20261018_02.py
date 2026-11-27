"""Recorded current-depth planning diagnostics only; no ROS/control."""
import json
from pathlib import Path
import sys
import numpy as np

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_auto_backend import read
from apple_transfer_auto_plan import footprint,depth_in_map,map_point,clear_patch
from tabletop_support import build_support,TableSupport,PLANNING_RESERVE_M
from hand_camera_ray import rotation
from inverse_table_place_plan import release_height
from pregrasp_geometry import expected_horizontal_center

out=Path('/__HANDYMAN_PACKAGE__/runtime/apple-navigation-feedback-r10-live-place-20261018-01')
config=read(out/'config.snapshot.json')
table=next(t for t in read(config['semantic_map'])['destinations'] if t['id']==config['destination_id'])
held=read('/__HANDYMAN_PACKAGE__/runtime/apple-gate-pruning-live-carry-place-20261018-01/verified-held-geometry.json')
results=[]
from measure_destination_surface import measure
for path in sorted((out/'destination-outside').glob('*/snapshot.json')):
    snapshot=read(path);depth=np.load(path.parent/'depth_m.npy',allow_pickle=False)
    plane=measure(depth,snapshot,table);coef=np.array(plane['plane_map'])
    points=depth_in_map(depth,snapshot);center,half=footprint(table)
    model=build_support(points,center,half,coef,table['id'],plane['source_depth_stamp_ns'])
    support=TableSupport(model,table['id']);positions=support.safe_centers(held['radius_m'],PLANNING_RESERVE_M)
    r=rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion']);rows=[];all_rows=[]
    for xy in positions:
        height=float(np.r_[xy,1.]@coef)
        palm_height=release_height(height,held['bottom_below_palm_m'],config['policy']['release_gap_m'])
        predicted=map_point(snapshot,expected_horizontal_center(palm_height,held['center_minus_palm_m']))
        delta=(r.T@np.r_[xy-predicted[:2],0.])[:2];distance=float(np.linalg.norm(delta))
        all_rows.append(dict(xy=xy.tolist(),distance_m=distance,delta_m=delta.tolist()))
        if delta[0]>0 and distance<=.6:
            rows.append(dict(xy=xy.tolist(),distance_m=distance,delta_m=delta.tolist(),
                **clear_patch(points,xy,coef,config['policy']['patch_radius_m'])))
    rows.sort(key=lambda v:v['distance_m'])
    all_rows.sort(key=lambda v:v['distance_m'])
    closest_support=[]
    for value in all_rows[:12]:
        closest_support.append(dict(value,**clear_patch(points,np.array(value['xy']),coef,config['policy']['patch_radius_m'])))
    item=dict(capture=str(path.parent),safe_support_positions=len(positions),reachable=len(rows),
        clear=sum(x['clear'] for x in rows),
        observed_free_eight_sector_candidates=sum(x['all_eight_sectors_observed'] for x in rows),
        obstacle_free_candidates=sum(x['obstacle_pixels']==0 for x in rows),
        closest_candidates=rows[:10],
        best_obstacle_count=min((x['obstacle_pixels'] for x in rows),default=None),
        highest_support_distance_m=float(np.max(support.distance_m)),
        nearest_safe_support=closest_support[:3],
        closest_twelve_clear=sum(v['clear'] for v in closest_support),
        best_candidates=sorted(rows,key=lambda v:(v['obstacle_pixels'],not v['all_eight_sectors_observed'],v['distance_m']))[:10])
    results.append(item)
result=dict(recorded_only=True,robot_commands_sent=False,frames=results)
with (out/'placement-failure-diagnosis-02.json').open('x') as f:json.dump(result,f,indent=2)
for item in results:print(json.dumps({k:v for k,v in item.items() if k not in ('closest_candidates','best_candidates')}))
print(json.dumps(dict(last_best_candidates=results[-1]['best_candidates'])),flush=True)

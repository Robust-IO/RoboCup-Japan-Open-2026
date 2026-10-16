"""Offline conservative disk path screening; no ROS, goals or object positions."""
import argparse
import ast
import hashlib
import heapq
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from analyze_search_connectivity import cell,world
from build_search_plan import inside


def route(allowed,start,goal):
    h,w=allowed.shape
    if any(not(0<=x<w and 0<=y<h) or not allowed[y,x] for x,y in (start,goal)):return None
    queue=[(0,start)];cost={start:0};parent={}
    while queue:
        _,p=heapq.heappop(queue)
        if p==goal:
            path=[p]
            while p!=start:p=parent[p];path.append(p)
            return path[::-1]
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            q=(p[0]+dx,p[1]+dy);x,y=q
            if 0<=x<w and 0<=y<h and allowed[y,x] and cost[p]+1<cost.get(q,math.inf):
                cost[q]=cost[p]+1;parent[q]=p
                heapq.heappush(queue,(cost[q]+abs(x-goal[0])+abs(y-goal[1]),q))
    return None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--layout',choices=['LayoutA','LayoutB','LayoutC','LayoutD'],required=True)
    p.add_argument('--room',required=True)
    a=p.parse_args();share=a.repo/'src/handyman_rebuild_ros2'
    env=yaml.safe_load((share/'config/environments'/('layout_'+a.layout[-1].lower()+'.yaml')).read_text())
    map_path=share/'maps'/a.layout/'map.yaml';content=map_path.read_bytes();meta=yaml.safe_load(content)
    if meta.get('mode','trinary')!='trinary':raise ValueError('Only trinary maps supported')
    image=map_path.parent/meta['image'];pixels=np.flipud(np.array(Image.open(image)))
    if pixels.ndim!=2 or pixels.dtype!=np.uint8:raise ValueError('Expected grayscale map')
    free=(pixels/255. if meta['negate'] else (255.-pixels)/255.)<meta['free_thresh']
    params=yaml.safe_load((a.repo/'src/handyman_ros2/param/nav2_params.yaml').read_text())
    cp=params['global_costmap']['global_costmap']['ros__parameters']
    footprint=ast.literal_eval(cp['footprint'])
    radius=max(math.hypot(x,y) for x,y in footprint)+float(cp.get('footprint_padding',.01))
    res=float(meta['resolution']);origin=meta['origin']
    # Include cell half diagonal, block unknown and outside-of-map cells.
    margin=radius+res/math.sqrt(2);n=math.ceil(margin/res)
    h,w=free.shape;padded=np.pad(free,n,constant_values=False);allowed=np.ones_like(free)
    for dy in range(-n,n+1):
        for dx in range(-n,n+1):
            if math.hypot(dx,dy)*res<=margin:
                allowed &= padded[n+dy:n+dy+h,n+dx:n+dx+w]
    start=cell(env['initial_pose']['x'],env['initial_pose']['y'],origin,res)
    room=env['rooms'][a.room];rows=[]
    for i,pose in enumerate(room['search_points']):
        path=route(allowed,start,cell(pose['x'],pose['y'],origin,res))
        rows.append(dict(point_id=f'{a.layout}/{a.room}/{i}',point_index=i,pose=pose,
                         in_room=inside(pose['x'],pose['y'],room['region']),
                         conservative_grid_path_found=path is not None,
                         grid_path_length_m=(len(path)-1)*res if path else None,
                         path_xy=[world(x,y,origin,res) for x,y in path] if path else []))
    candidates=[r for r in rows if r['in_room'] and r['conservative_grid_path_found']]
    selected=min(candidates,key=lambda r:r['grid_path_length_m']) if candidates else None
    report=dict(layout=a.layout,room=a.room,initial_pose=env['initial_pose'],
                footprint_disk_radius_m=radius,grid_margin_m=margin,
                map_bundle_sha256=hashlib.sha256(content+b'\0'+image.read_bytes()).hexdigest(),
                candidates=rows,suggested_point_id=selected['point_id'] if selected else None,
                actionable=False,nav2_path_verified=False,
                limitations=['Configured initial pose, not live pose','Conservative 4-connected raster path, not Nav2 plan',
                             'No dynamic obstacle check','No target visibility or camera coverage claim'])
    with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps({**{k:v for k,v in report.items() if k!='candidates'},
                     'candidates':[{k:v for k,v in r.items() if k!='path_xy'} for r in rows]}))


if __name__=='__main__':main()

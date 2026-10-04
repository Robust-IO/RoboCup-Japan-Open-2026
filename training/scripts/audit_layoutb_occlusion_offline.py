"""Read saved maps/captures only. No ROS, network, model loading or motion."""
import argparse
from collections import Counter
import json
from pathlib import Path
import math
import ast
import hashlib

import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_search_connectivity import cell, world
from build_active_search_points import disk_free, distances
from search_room_gate import RoomGate

def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--observer',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--all-points',type=Path)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads(args.bundle.read_text())
    share=args.bundle.parent/'package'
    env=yaml.safe_load((share/'config/environments/layout_b.yaml').read_text())
    meta=yaml.safe_load((share/'maps/LayoutB/map.yaml').read_text())
    pixels=np.flipud(np.array(Image.open(share/'maps/LayoutB'/meta['image'])))
    probability=pixels/255. if meta['negate'] else (255.-pixels)/255.
    free=probability<meta['free_thresh'];res=float(meta['resolution']);origin=meta['origin']
    if abs(origin[2])>1e-9:raise ValueError('plot requires zero map yaw')
    poly=env['rooms']['living_room']['region']
    main=manifest['main'];backup=manifest['supplement']['proposed_backups'][0]
    old={'x':-2.135,'y':-6.645}
    params=yaml.safe_load((Path(manifest['source_repo'])/'src/handyman_ros2/param/nav2_params.yaml').read_text())
    param_path=Path(manifest['source_repo'])/'src/handyman_ros2/param/nav2_params.yaml'
    if hashlib.sha256(param_path.read_bytes()).hexdigest()!=manifest['footprint_config_sha256']:
        raise ValueError('footprint changed since saved bundle')
    cost=params['global_costmap']['global_costmap']['ros__parameters']
    footprint=ast.literal_eval(cost['footprint']) if isinstance(cost['footprint'],str) else cost['footprint']
    radius=max(math.hypot(*p) for p in footprint)+float(cost.get('footprint_padding',.01))
    allowed=disk_free(free,res,radius)
    start=cell(main['x'],main['y'],origin,res);end=cell(backup['x'],backup['y'],origin,res)
    distance=distances(allowed,start);path=[]
    x,y=end
    if distance[y,x]>=0:
        path=[world(x,y,origin,res)]
        while (x,y)!=start:
            choices=[(xx,yy) for xx,yy in ((x-1,y),(x+1,y),(x,y-1),(x,y+1))
                     if 0<=yy<free.shape[0] and 0<=xx<free.shape[1] and distance[yy,xx]==distance[y,x]-1]
            x,y=choices[0];path.append(world(x,y,origin,res))
    h,w=free.shape;extent=[origin[0],origin[0]+w*res,origin[1],origin[1]+h*res]
    fig,ax=plt.subplots(figsize=(9,12))
    ax.imshow(free,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=1)
    if path:
        route=np.array(path);ax.plot(route[:,0],route[:,1],color='purple',linewidth=1.5,label='Offline grid path (not Nav2)')
    polygon=np.array(poly+[poly[0]]);ax.plot(polygon[:,0],polygon[:,1],'b--',label='Living room boundary')
    for name,pose,color in [('Main',main,'green'),('Previous backup',old,'orange'),('Interior backup',backup,'red')]:
        ax.scatter(pose['x'],pose['y'],color=color,s=80,label=name)
        ax.annotate(name,(pose['x'],pose['y']),xytext=(8,8),textcoords='offset points')
        if 'yaw' in pose:ax.arrow(pose['x'],pose['y'],.4*math.cos(pose['yaw']),.4*math.sin(pose['yaw']),color=color,width=.02)
    ax.set_xlim(min(x for x,y in poly)-.5,max(x for x,y in poly)+.5)
    ax.set_ylim(min(y for x,y in poly)-.5,max(y for x,y in poly)+.5)
    ax.set_aspect('equal');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
    ax.set_title('LayoutB: saved occupancy map / proposed points\nNot a 3-D visibility or live navigation proof')
    ax.legend(loc='upper right',fontsize=8);fig.tight_layout();fig.savefig(args.output/'layoutb-points.png',dpi=150);plt.close(fig)

    observed=records(args.observer)
    stamps=[r['motion']['stamp_ns'] for r in observed if r.get('state')=='observe' and r.get('motion',{}).get('stamp_ns')]
    if not stamps:raise ValueError('no saved observation stamps')
    lo,hi=min(stamps),max(stamps)
    arrivals=[r.get('view_evidence',{}).get('arrival_stamp_ns',0) for r in observed]
    lo=max(lo,max(arrivals,default=0))
    diagnostics=records(args.capture/'diagnostics.jsonl')
    rows=[r for r in diagnostics if lo<=r.get('depth_stamp_ns',0)<=hi and 'sample' in r]
    unique={r['sample']:r for r in rows};rows=list(unique.values())
    detail=[]
    for r in rows:
        detections=[d for d in r.get('inference',{}).get('detections',[]) if d.get('name')=='filled_ketchup']
        detail.append(dict(sample=r['sample'],depth_stamp_ns=r['depth_stamp_ns'],quality=r['quality'],detections=detections))
    reasons=Counter(r['quality'].get('reason',r['quality'].get('status')) for r in rows)
    scores=[d['confidence'] for r in detail for d in r['detections']]
    hashes=[hashlib.sha256((args.capture/f"{r['sample']:06d}"/'rgb.png').read_bytes()).hexdigest() for r in rows]
    batch=[]
    if args.all_points:
        generated=json.loads(args.all_points.read_text())
        for layout in generated['reports']:
            config=Path(manifest['source_repo'])/'src/handyman_rebuild_ros2/config/environments'/('layout_'+layout['layout'][-1].lower()+'.yaml')
            if hashlib.sha256(config.read_bytes()).hexdigest()!=layout['environment_sha256']:
                raise ValueError('environment changed since batch generation')
            e=yaml.safe_load(config.read_text())
            for room in layout['rooms']:
                gate=RoomGate({n:r['region'] for n,r in e['rooms'].items()},room['room'])
                points=([room['main']] if room['main'] else [])+room['backups']
                checks=[gate.classify([v['x'],v['y'],0],[0,0,0],[0,0,0,1])['state'] for v in points]
                batch.append(dict(layout=layout['layout'],room=room['room'],status=room['status'],
                    backup_count=len(room['backups']),membership=checks,
                    separation_ok=all(math.hypot(a['x']-b['x'],a['y']-b['y'])>=1. for i,a in enumerate(points) for b in points[i+1:]),
                    does_not_exist_authorized=room['does_not_exist_authorized']))
    selected=[]
    for reason in reasons:
        group=[r for r in rows if r['quality'].get('reason',r['quality'].get('status'))==reason]
        selected.extend([group[0],group[-1]])
    selected=sorted({r['sample']:r for r in selected}.values(),key=lambda r:r['sample'])[:12]
    if selected:
        fig,axes=plt.subplots(len(selected),2,figsize=(14,4*len(selected)),squeeze=False)
        for pair,r in zip(axes,selected):
            folder=args.capture/f"{r['sample']:06d}"
            for ax,name in zip(pair,['rgb.png','detection.jpg']):
                image_path=folder/name
                if image_path.exists():ax.imshow(Image.open(image_path))
                ax.set_title(f"{r['sample']} {name}: {r['quality'].get('reason',r['quality'].get('status'))}")
                ax.axis('off')
        fig.tight_layout();fig.savefig(args.output/'observation-samples.jpg',dpi=100);plt.close(fig)
    report=dict(source_observer=str(args.observer),capture=str(args.capture),
        observation_stamp_range=[lo,hi],unique_frames=len(rows),quality_counts=dict(reasons),
        confidence_range=[min(scores),max(scores)] if scores else [],
        unique_rgb_file_hashes=len(set(hashes)),
        unique_depth_stamps=len({r['depth_stamp_ns'] for r in rows}),
        all_map_checks=batch,
        offline_path_found=bool(path),offline_path_length_m=int(distance[end[1],end[0]])*res if path else None,
        adapter_counts=dict(Counter(r['adapter_reason'] for r in observed if r.get('state')=='observe' and 'adapter_reason' in r)),
        frames=detail,main=main,backup=backup,live_started=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='frames'},indent=2))


if __name__=='__main__':main()

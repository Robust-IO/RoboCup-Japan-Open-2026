"""Offline point-agent connectivity screening. Not robot reachability proof."""
import argparse,hashlib,json,math
from collections import deque
from pathlib import Path
import numpy as np
from build_search_plan import inside

def flood(allowed,start):
    visited=np.zeros(allowed.shape,dtype=bool)
    x,y=start; h,w=allowed.shape
    if not (0<=x<w and 0<=y<h) or not allowed[y,x]: return visited
    visited[y,x]=True; queue=deque([(x,y)])
    while queue:
        x,y=queue.popleft()
        # Optimistic diagonal connectivity: no false impossibility due to corners.
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            xx,yy=x+dx,y+dy
            if 0<=xx<w and 0<=yy<h and allowed[yy,xx] and not visited[yy,xx]:
                visited[yy,xx]=True; queue.append((xx,yy))
    return visited

def cell(x,y,origin,resolution):
    dx,dy=x-origin[0],y-origin[1]; c,s=math.cos(origin[2]),math.sin(origin[2])
    return math.floor((c*dx+s*dy)/resolution),math.floor((-s*dx+c*dy)/resolution)

def world(x,y,origin,resolution):
    xx,yy=(x+.5)*resolution,(y+.5)*resolution;c,s=math.cos(origin[2]),math.sin(origin[2])
    return origin[0]+c*xx-s*yy,origin[1]+s*xx+c*yy

def analyze(environment,map_path):
    import yaml
    from PIL import Image
    meta=yaml.safe_load(map_path.read_text())
    if meta.get('mode','trinary')!='trinary': raise ValueError('Only trinary maps supported')
    res=float(meta['resolution']); origin=list(map(float,meta['origin']))
    if res<=0 or len(origin)!=3 or not all(math.isfinite(v) for v in [res,*origin]): raise ValueError('invalid geometry')
    image_path=map_path.parent/meta['image']; pixels=np.array(Image.open(image_path))
    if pixels.ndim!=2 or pixels.dtype!=np.uint8: raise ValueError('Only 8-bit grayscale maps supported')
    pixels=np.flipud(pixels)
    prob=pixels/255. if meta['negate'] else (255.-pixels)/255.
    free=prob<float(meta['free_thresh']); occupied=prob>float(meta['occupied_thresh']); unknown=~(free|occupied)
    start=cell(environment['initial_pose']['x'],environment['initial_pose']['y'],origin,res)
    known=flood(free,start); optimistic=flood(~occupied,start)
    initial_valid=bool(known.any())
    rooms=[]
    for name,room in environment['rooms'].items():
        polygon=[tuple(p) for p in room['region']]
        counts={'known_free':0,'known_connected':0,'optimistic_traversable':0,'optimistic_connected':0}
        for y,x in zip(*np.where(~occupied)):
            if inside(*world(int(x),int(y),origin,res),polygon):
                counts['known_free']+=int(free[y,x]);counts['known_connected']+=int(known[y,x])
                counts['optimistic_traversable']+=1;counts['optimistic_connected']+=int(optimistic[y,x])
        if not initial_valid: status='invalid_initial_cell'
        elif counts['known_connected']: status='point_connectivity_present'
        elif counts['optimistic_connected']: status='unknown_space_requires_review'
        elif not counts['optimistic_traversable']: status='no_room_traversable_cells'
        else: status='structural_disconnection_candidate'
        points=[]
        for p in room.get('search_points',[]):
            x,y=cell(p['x'],p['y'],origin,res); valid=0<=y<free.shape[0] and 0<=x<free.shape[1]
            points.append(dict(pose=p,in_bounds=valid,known_connected=bool(valid and known[y,x]),optimistic_connected=bool(valid and optimistic[y,x])))
        rooms.append(dict(room=name,status=status,cells=counts,search_points=points))
    digest=hashlib.sha256(map_path.read_bytes()+b'\0'+image_path.read_bytes()).hexdigest()
    return dict(layout=environment['internal_name'],map_bundle_sha256=digest,initial_cell=start,
                initial_known_free=initial_valid,unknown_cells=int(unknown.sum()),rooms=rooms,
                method='optimistic_8_connected_point_agent',actionable=False,structural_evidence_approved=False,
                limitations=['No robot footprint inflation.','Rasterisation and semantic boundaries need review.',
                             'Uses configured initial pose, not live robot pose.','Not accepted by unreachable_policy as structural proof.'])

def main():
    import yaml
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); package=a.repo/'src/handyman_rebuild_ros2'; reports=[]
    for path in sorted((package/'config/environments').glob('layout_*.yaml')):
        env=yaml.safe_load(path.read_text()); ref=env['map']; prefix='package://handyman_rebuild_ros2/'
        if not ref.startswith(prefix): raise ValueError('Unexpected map reference')
        report=analyze(env,package/ref[len(prefix):]); reports.append(report)
        print(report['layout'],report['initial_known_free'],[(r['room'],r['status'],sum(x['known_connected'] for x in r['search_points'])) for r in report['rooms']],flush=True)
    with a.output.open('x') as f: json.dump(dict(reports=reports),f,indent=2)
    print('Saved',a.output)

if __name__=='__main__': main()

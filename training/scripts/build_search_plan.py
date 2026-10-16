"""Offline room search-point manifest. No route execution or coverage claim."""
import argparse
import hashlib
import json
import math
from pathlib import Path

def inside(x,y,polygon):
    hit=False
    for (ax,ay),(bx,by) in zip(polygon,polygon[1:]+polygon[:1]):
        cross=(x-ax)*(by-ay)-(y-ay)*(bx-ax)
        if abs(cross)<1e-8 and min(ax,bx)-1e-8<=x<=max(ax,bx)+1e-8 and min(ay,by)-1e-8<=y<=max(ay,by)+1e-8:
            return True
        if (ay>y)!=(by>y) and x<(bx-ax)*(y-ay)/(by-ay)+ax: hit=not hit
    return hit

def build(config):
    name=config.get('internal_name')
    if not isinstance(name,str) or not name: raise ValueError('missing_layout_name')
    if not isinstance(config.get('rooms'),dict) or not config['rooms']: raise ValueError('missing_rooms')
    rooms=[]
    for room,data in config['rooms'].items():
        errors=[]; points=[]; seen=set()
        try:
            polygon=[tuple(float(v) for v in p) for p in data['region']]
            if len(polygon)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in polygon): raise ValueError()
            area=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(polygon,polygon[1:]+polygon[:1]))
            if abs(area)<1e-8: raise ValueError()
        except (KeyError,TypeError,ValueError):
            polygon=[]; errors.append('invalid_room_region')
        raw=data.get('search_points',[]) if isinstance(data,dict) else []
        if not isinstance(raw,list) or not raw:
            errors.append('missing_search_points'); raw=[]
        for index,p in enumerate(raw):
            issues=[]
            try:
                pose={k:float(p[k]) for k in ('x','y','yaw')}
                if not all(math.isfinite(v) for v in pose.values()): raise ValueError()
                pose['yaw']=math.atan2(math.sin(pose['yaw']),math.cos(pose['yaw']))
                key=tuple(round(pose[k],6) for k in ('x','y','yaw'))
                if key in seen: issues.append('duplicate_pose')
                seen.add(key)
                if not polygon or not inside(pose['x'],pose['y'],polygon): issues.append('outside_or_invalid_region')
            except (KeyError,TypeError,ValueError,OverflowError):
                pose=None; issues.append('invalid_pose')
            points.append(dict(id=f'{name}/{room}/{index}',source_index=index,pose=pose,issues=issues))
        rooms.append(dict(room=room,valid=not errors and all(not p['issues'] for p in points),errors=errors,points=points))
    return dict(schema='handyman-search-plan-v1',layout=name,frame_id='map',rooms=rooms,
                actionable=False,coverage_verified=False,does_not_exist_authorized=False,
                limitations=['Region membership is not collision checking or reachability.','No head motion generated.','Point order preserves YAML order; no route optimisation.'])

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environments',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import yaml
    paths=sorted(args.environments.glob('layout_*.yaml'))
    if not paths: parser.error('No layout YAML found')
    plans=[]
    for path in paths:
        content=path.read_bytes()
        plan=build(yaml.safe_load(content))
        plan.update(source=str(path),source_sha256=hashlib.sha256(content).hexdigest())
        plans.append(plan)
    with args.output.open('x') as stream: json.dump({'plans':plans},stream,indent=2,allow_nan=False)
    for p in plans:
        print(p['layout'],[(r['room'],len(r['points']),r['valid'],r['errors'],[x['issues'] for x in r['points'] if x['issues']]) for r in p['rooms']])
    print('Saved:',args.output,'; no navigation commands')

if __name__=='__main__': main()

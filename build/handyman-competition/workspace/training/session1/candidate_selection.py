"""Rank exported poses by static traversability, sight line, then grid path length.

This estimates paths on the existing map. Nav2 and live sensing remain responsible
for actual navigation and moving obstacles. No navigation parameters are changed.
"""
import heapq
import math
from pathlib import Path
import numpy as np

def destination_candidates(candidates,config,region=None):
    """Optional per-destination approach along the exported facing direction."""
    spec=config.get('session_profile',{}).get('destination',{})
    shift=float(spec.get('candidate_forward_shift_m',0.))
    if not math.isfinite(shift) or shift<0 or shift>.3:raise ValueError('invalid_candidate_forward_shift')
    result=[[float(p[0]+shift*math.cos(p[2])),float(p[1]+shift*math.sin(p[2])),p[2]] for p in candidates]
    if spec.get('mode')!='container_drop':return result
    opening=config.get('container_opening',{})
    if opening.get('destination_id')!=config.get('destination_id'):
        raise ValueError('container_candidate_destination_mismatch')
    polygon=np.asarray(opening['polygon_map_m'],float)
    if polygon.ndim!=2 or polygon.shape[1]!=2 or len(polygon)<3 or not np.isfinite(polygon).all():
        raise ValueError('invalid_container_candidate_opening')
    center=polygon.mean(axis=0)
    # Rotate existing staging offsets around the actual mouth. Preserve standoff;
    # no preference for the model's front. The selector rejects blocked paths/views.
    for pose in list(result):
        offset=np.asarray(pose[:2])-center
        for angle in (math.pi/2,-math.pi/2,math.pi):
            c,s=math.cos(angle),math.sin(angle)
            xy=center+np.array([[c,-s],[s,c]])@offset
            result.append([float(xy[0]),float(xy[1]),math.atan2(center[1]-xy[1],center[0]-xy[0])])
    from perception_bridge import inside
    unique=[]
    for pose in result:
        if region is not None and not inside(*pose[:2],region):continue
        if not any(math.dist(pose[:2],p[:2])<.01 for p in unique):unique.append(pose)
    return unique

class CandidateSelector:
    def __init__(self,free,resolution,origin,radius,tables):
        from scipy.ndimage import distance_transform_edt
        self.free=np.asarray(free,bool);self.res=float(resolution);self.origin=origin
        self.radius=float(radius);self.tables=tables
        # Include map boundary as occupied when computing footprint clearance.
        padded=np.pad(self.free,1,constant_values=False)
        clearance=distance_transform_edt(padded)[1:-1,1:-1]*self.res
        self.walk=self.free & (clearance>self.radius)
    def cell(self,p):
        x,y=np.asarray(p[:2])-self.origin[:2];a=self.origin[2]
        return (math.floor((-math.sin(a)*x+math.cos(a)*y)/self.res),
                math.floor((math.cos(a)*x+math.sin(a)*y)/self.res))
    def valid(self,c):return 0<=c[0]<self.free.shape[0] and 0<=c[1]<self.free.shape[1]
    def lengths(self,start,goals):
        if not self.valid(start) or not self.walk[start]:return {}
        pending=set(goals);dist={start:0.};queue=[(0.,start)];found={}
        while queue and pending:
            d,c=heapq.heappop(queue)
            if d!=dist[c]:continue
            if c in pending:found[c]=d;pending.remove(c)
            for dy,dx in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                n=(c[0]+dy,c[1]+dx)
                if not self.valid(n) or not self.walk[n]:continue
                if dx and dy and (not self.walk[c[0]+dy,c[1]] or not self.walk[c[0],c[1]+dx]):continue
                cost=d+self.res*math.hypot(dx,dy)
                if cost<dist.get(n,math.inf):dist[n]=cost;heapq.heappush(queue,(cost,n))
        return found
    def clear_view(self,pose,table):
        bounds=table['axis_aligned_bounds'];center=bounds['center'];size=bounds['size']
        target=np.array([center['x'],center['y']]);start=np.array(pose[:2])
        count=max(2,math.ceil(np.linalg.norm(target-start)/(self.res*.4)))
        for point in np.linspace(start,target,count):
            # The intended support furniture is not an intervening obstacle.
            # Map rasterization may occupy the cell intersecting the support edge.
            edge_tolerance=self.res*math.sqrt(2)
            if abs(point[0]-target[0])<=size['x']/2+edge_tolerance and abs(point[1]-target[1])<=size['y']/2+edge_tolerance:break
            cell=self.cell(point)
            if not self.valid(cell) or not self.free[cell]:return False
        return True
    def rank(self,pose,candidates,furniture_id):
        table=self.tables[furniture_id];cells=[self.cell(c) for c in candidates]
        distances=self.lengths(self.cell(pose),cells);rows=[]
        for index,(candidate,cell) in enumerate(zip(candidates,cells)):
            clear=self.clear_view(candidate,table)
            length=distances.get(cell)
            reason=('unreachable_on_static_footprint_grid' if length is None else
                    'view_blocked_on_static_map' if not clear else None)
            rows.append(dict(index=index,pose=candidate,path_length_m=length,view_clear=clear,rejected_reason=reason))
        accepted=sorted((r for r in rows if r['rejected_reason'] is None),key=lambda r:(r['path_length_m'],r['index']))
        return [r['pose'] for r in accepted],dict(method='static_footprint_grid_path_and_support_sightline',
            footprint_radius_m=self.radius,candidates=rows,ranked_indices=[r['index'] for r in accepted],
            dynamic_obstacles_evaluated=False)

def load_selector(config,semantic):
    import yaml
    from PIL import Image
    environment=Path(config['environment']);env=yaml.safe_load(environment.read_text())
    uri=env['map']
    if uri.startswith('package://'):
        package,relative=uri[len('package://'):].split('/',1)
        package_root=next((p for p in environment.parents if p.name==package),None)
        if package_root is None:
            from ament_index_python.packages import get_package_share_directory
            package_root=Path(get_package_share_directory(package))
        path=package_root/relative
    else:path=environment.parent/uri
    info=yaml.safe_load(path.read_text());pixels=np.asarray(Image.open(path.parent/info['image']).convert('L'),float)/255
    occupancy=pixels if info.get('negate',0) else 1-pixels
    free=np.flipud(occupancy<float(info['free_thresh']))
    params=Path(config.get('navigation_params','/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/handyman_ros2/share/handyman_ros2/param/nav2_params.yaml'))
    nav=yaml.safe_load(params.read_text())
    local=nav['local_costmap']['local_costmap']['ros__parameters']
    footprint=local.get('footprint')
    if footprint:
        points=yaml.safe_load(footprint) if isinstance(footprint,str) else footprint
        radius=max(math.hypot(*p) for p in points)+float(local.get('footprint_padding',.01))
    else:radius=float(local['robot_radius'])+float(local.get('footprint_padding',.01))
    return CandidateSelector(free,info['resolution'],info['origin'],radius,{t['id']:t for t in semantic['destinations']})

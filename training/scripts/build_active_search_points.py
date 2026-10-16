"""Offline conservative main-point proposals; never edits maps or sends goals."""
import argparse
import ast
from collections import deque
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from active_search_policy import ring_views
from analyze_search_connectivity import cell, world
from build_search_plan import inside
from search_visibility import choose_backups
from search_room_gate import RoomGate


def disk_free(free, resolution, radius):
    if (free.ndim != 2 or free.dtype != bool or not math.isfinite(resolution)
            or not math.isfinite(radius) or resolution <= 0 or radius < 0):
        raise ValueError('invalid_grid_geometry')
    # Account for the full area of blocked raster cells, not just their centers.
    margin = radius + resolution / math.sqrt(2)
    n = math.ceil(margin / resolution)
    h, w = free.shape
    padded = np.pad(free, n, constant_values=False)
    allowed = np.ones_like(free)
    for dy in range(-n, n+1):
        for dx in range(-n, n+1):
            if math.hypot(dx, dy)*resolution <= margin:
                allowed &= padded[n+dy:n+dy+h, n+dx:n+dx+w]
    return allowed


def distances(allowed, start):
    result = np.full(allowed.shape, -1, dtype=np.int32)
    x, y = start
    h, w = allowed.shape
    if not (0 <= x < w and 0 <= y < h and allowed[y, x]):
        return result
    queue = deque([(x, y)])
    result[y, x] = 0
    while queue:
        x, y = queue.popleft()
        for xx, yy in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if 0 <= xx < w and 0 <= yy < h and allowed[yy,xx] and result[yy,xx] < 0:
                result[yy,xx] = result[y,x]+1
                queue.append((xx,yy))
    return result


def select_points(free, resolution, origin, start_xy, rooms, radius):
    if len(origin) != 3 or not all(math.isfinite(v) for v in (*origin,*start_xy)):
        raise ValueError('invalid_origin_or_start')
    allowed = disk_free(free, resolution, radius)
    extra_clearance = disk_free(free, resolution, radius+.15)
    distance = distances(allowed, cell(*start_xy, origin, resolution))
    start_valid = bool(np.any(distance >= 0))
    reports = []
    for name, room in rooms.items():
        polygon = [tuple(float(v) for v in p) for p in room['region']]
        if len(polygon)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in polygon):
            raise ValueError('invalid_room_polygon')
        area = sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(polygon,polygon[1:]+polygon[:1]))
        if abs(area)<1e-8:
            raise ValueError('degenerate_room_polygon')
        # This center is only a ranking preference, never an assumed free pose.
        cx = (min(p[0] for p in polygon)+max(p[0] for p in polygon))/2
        cy = (min(p[1] for p in polygon)+max(p[1] for p in polygon))/2
        candidates = []
        for y,x in zip(*np.where(distance >= 0)):
            wx,wy = world(int(x),int(y),origin,resolution)
            if not inside(wx,wy,polygon):
                continue
            path_m = int(distance[y,x])*resolution
            center_m = math.hypot(wx-cx,wy-cy)
            score = center_m + .15*path_m + (0 if extra_clearance[y,x] else .4)
            candidates.append((score,path_m,wx,wy,bool(extra_clearance[y,x])))
        candidates.sort()
        selected = []
        for score,path_m,x,y,extra in candidates[:1]:
            yaw = math.atan2(cy-y,cx-x) if math.hypot(cx-x,cy-y)>1e-6 else 0.
            selected.append(dict(x=x,y=y,yaw=yaw,ranking_cost=score,
                                 conservative_path_length_m=path_m,extra_clearance=extra))
        if selected:
            room_mask = np.zeros_like(free)
            for yy,xx in zip(*np.where(free)):
                wx,wy = world(int(xx),int(yy),origin,resolution)
                room_mask[yy,xx] = inside(wx,wy,polygon)
            gate = RoomGate({n:r['region'] for n,r in rooms.items()},name)
            safe_candidates = [c for c in candidates if gate.classify(
                [c[2],c[3],0],[0,0,0],[0,0,0,1])['state']=='inside']
            selected.extend(choose_backups(free,allowed,resolution,origin,
                safe_candidates,selected[0],room_mask,distances))
        status = ('proposed' if selected else 'no_conservative_candidate') if start_valid else 'start_requires_review'
        reports.append(dict(room=name,status=status,main=selected[0] if selected else None,
                            backups=selected[1:],candidate_count=len(candidates),
                            views=ring_views(selected[0]['x'],selected[0]['y'],selected[0]['yaw']) if selected else [],
                            actionable=False,does_not_exist_authorized=False))
    return reports


def build(repo):
    import yaml
    from PIL import Image
    share=repo/'src/handyman_rebuild_ros2'
    params_path=repo/'src/handyman_ros2/param/nav2_params.yaml'
    params=yaml.safe_load(params_path.read_text())
    cp=params['global_costmap']['global_costmap']['ros__parameters']
    footprint=ast.literal_eval(cp['footprint']) if isinstance(cp['footprint'],str) else cp['footprint']
    if len(footprint)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in footprint):
        raise ValueError('invalid_footprint')
    padding=float(cp.get('footprint_padding',.01))
    if not math.isfinite(padding) or padding<0:
        raise ValueError('invalid_padding')
    radius=max(math.hypot(*p) for p in footprint)+padding
    reports=[]
    for path in sorted((share/'config/environments').glob('layout_*.yaml')):
        env=yaml.safe_load(path.read_text())
        prefix='package://handyman_rebuild_ros2/'
        if not env['map'].startswith(prefix):
            raise ValueError('unsupported_map_reference')
        map_path=share/env['map'][len(prefix):]
        meta=yaml.safe_load(map_path.read_text())
        if meta.get('mode','trinary')!='trinary' or meta['negate'] not in (0,1):
            raise ValueError('unsupported_map_mode')
        if not 0<=meta['free_thresh']<meta['occupied_thresh']<=1:
            raise ValueError('invalid_map_thresholds')
        image_path=map_path.parent/meta['image']
        pixels=np.flipud(np.array(Image.open(image_path)))
        if pixels.ndim!=2 or pixels.dtype!=np.uint8:
            raise ValueError('expected_grayscale_map')
        probability=pixels/255. if meta['negate'] else (255.-pixels)/255.
        free=probability<meta['free_thresh']
        start=env['initial_pose']
        reports.append(dict(layout=env['internal_name'],
            map_bundle_sha256=hashlib.sha256(map_path.read_bytes()+b'\0'+image_path.read_bytes()).hexdigest(),
            environment_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            rooms=select_points(free,float(meta['resolution']),meta['origin'],(start['x'],start['y']),env['rooms'],radius)))
    if not reports:
        raise ValueError('no_environment_maps')
    return dict(schema='handyman-active-search-points-v1',reports=reports,
                footprint_radius_m=radius,footprint_config_sha256=hashlib.sha256(params_path.read_bytes()).hexdigest(),
                actionable=False,does_not_exist_authorized=False,
                limitations=['Uses configured initial pose, not current robot pose.',
                             'Circumscribed-disk four-connected paths are conservative; rejection does not prove disconnection.',
                             'No dynamic obstacles, furniture visibility or target coverage verified.',
                             'Backup gain is a room-clipped 2-D occupancy proxy; furniture height and actual RGBD visibility remain unverified.',
                             'Live Nav2 path and full current robot configuration must be checked before execution.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=build(args.repo)
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
    for layout in report['reports']:
        print(layout['layout'],[(r['room'],r['status'],r['main']) for r in layout['rooms']],flush=True)


if __name__=='__main__':main()

"""Offline 2-D occlusion proxy, never a measured coverage/absence proof."""
import math
from collections import deque
import numpy as np
from analyze_search_connectivity import cell, world


def visible_cells(free, source, targets):
    """Conservative sampled supercover: unknown/non-free and corner touches block."""
    targets = np.asarray(targets, dtype=int).reshape((-1, 2))
    if not len(targets):
        return np.zeros(0, dtype=bool)
    source = np.asarray(source, dtype=int)
    delta = targets-source
    steps = max(1, int(np.max(np.abs(delta)))*2)
    visible = np.ones(len(targets), dtype=bool)
    for t in np.linspace(0, 1, steps+1):
        p = source + delta*t
        lo = np.floor(p).astype(int); hi = np.ceil(p).astype(int)
        for x, y in ((lo[:,0],lo[:,1]),(lo[:,0],hi[:,1]),
                     (hi[:,0],lo[:,1]),(hi[:,0],hi[:,1])):
            visible &= free[y,x]
    return visible


def region_interiors(mask):
    """Four-connected hidden regions and distance from every region boundary.

    Distance is in grid steps, not a Euclidean clearance guarantee. The deepest
    cells are reachable interior representatives, not polygon centroids.
    """
    h,w=mask.shape
    labels=np.zeros(mask.shape,dtype=np.int32)
    depth=np.zeros(mask.shape,dtype=np.int32)
    sizes={}; number=0; boundary=deque()
    for y,x in zip(*np.where(mask)):
        if labels[y,x]:continue
        number+=1; queue=deque([(int(x),int(y))]);labels[y,x]=number;size=0
        while queue:
            xx,yy=queue.popleft();size+=1;edge=False
            for nx,ny in ((xx-1,yy),(xx+1,yy),(xx,yy-1),(xx,yy+1)):
                if not (0<=nx<w and 0<=ny<h) or not mask[ny,nx]:
                    edge=True
                elif not labels[ny,nx]:
                    labels[ny,nx]=number;queue.append((nx,ny))
            if edge:depth[yy,xx]=1;boundary.append((xx,yy))
        sizes[number]=size
    while boundary:
        x,y=boundary.popleft()
        for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if 0<=nx<w and 0<=ny<h and mask[ny,nx] and not depth[ny,nx]:
                depth[ny,nx]=depth[y,x]+1;boundary.append((nx,ny))
    return labels,depth,sizes


def choose_backups(free, allowed, resolution, origin, candidates, main, room_mask,
                   distance_fn, maximum=2):
    # Equal-area samples include free space behind furniture, even where the
    # robot cannot stand. Rays must remain inside the target room.
    stride = max(1, round(.35/resolution))
    ys, xs = np.where(free & room_mask)
    use = (xs % stride == 0) & (ys % stride == 0)
    targets = np.column_stack((xs[use],ys[use]))
    ray_free = free & room_mask
    seen = visible_cells(ray_free, cell(main['x'],main['y'],origin,resolution), targets)
    # Interior candidates use full map resolution, not half-metre bins.
    selected = [main]; result = []
    for _ in range(maximum):
        last = selected[-1]
        path = distance_fn(allowed,cell(last['x'],last['y'],origin,resolution))
        hidden=np.zeros_like(free)
        cy,cx=np.where(allowed & room_mask)
        cells=np.column_stack((cx,cy))
        covered=np.zeros(len(cells),dtype=bool)
        for p in selected:
            covered |= visible_cells(ray_free,cell(p['x'],p['y'],origin,resolution),cells)
        hidden[cy,cx]=~covered
        labels,depth,sizes=region_interiors(hidden)
        eligible={}
        for c in candidates:
            ix,iy=cell(c[2],c[3],origin,resolution)
            label=int(labels[iy,ix])
            if (not label or sizes[label]*resolution**2 < .5 or path[iy,ix]<0
                    or any(math.hypot(c[2]-p['x'],c[3]-p['y'])<1 for p in selected)):
                continue
            eligible.setdefault(label,[]).append(c)
        core=[]
        for label,rows in eligible.items():
            deepest=max(int(depth[cell(c[2],c[3],origin,resolution)[1],
                                  cell(c[2],c[3],origin,resolution)[0]]) for c in rows)
            if deepest*resolution < .25:continue
            for c in rows:
                ix,iy=cell(c[2],c[3],origin,resolution)
                if depth[iy,ix]==deepest:
                    core.append((c,visible_cells(ray_free,(ix,iy),targets)))
        options = []
        for c, visible in core:
            _, _, x, y, _ = c
            if any(math.hypot(x-p['x'],y-p['y']) < 1. for p in selected):
                continue
            ix,iy = cell(x,y,origin,resolution)
            if path[iy,ix] < 0:
                continue
            gain = visible & ~seen
            area = int(gain.sum())*(stride*resolution)**2
            if area < .5:
                continue
            travel = int(path[iy,ix])*resolution
            # Gain dominates; travel penalizes long detours, not Euclidean gaps.
            utility = area/(1+.10*travel)
            options.append((utility,area,-travel,c,visible,gain))
        if not options:
            break
        _,area,negtravel,c,visible,gain = max(options,key=lambda o:(o[0],o[1],o[2],-o[3][0]))
        _,path_m,x,y,extra = c
        ix,iy=cell(x,y,origin,resolution)
        center = np.mean(targets[gain],axis=0)
        gx,gy = world(float(center[0]),float(center[1]),origin,resolution)
        p = dict(x=x,y=y,yaw=math.atan2(gy-y,gx-x),ranking_cost=c[0],
                 conservative_path_length_m=path_m,extra_clearance=extra,
                 estimated_new_visible_area_m2=area,transition_path_length_m=-negtravel,
                 selection_strategy='hidden_region_interior',
                 hidden_region_area_m2=sizes[int(labels[iy,ix])]*resolution**2,
                 interior_depth_grid_m=int(depth[iy,ix])*resolution,
                 visibility_model='room_clipped_2d_occupancy_proxy',coverage_verified=False)
        result.append(p); selected.append(p); seen |= visible
    return result

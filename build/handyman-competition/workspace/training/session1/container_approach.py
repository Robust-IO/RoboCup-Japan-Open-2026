"""Find a reachable approach region around the actual mouth, not exported poses."""
import math
import numpy as np
from perception_bridge import inside

def plan_approach(selector,pose,opening,room_polygon,furniture_id,*,target_bounds=None,handoff_radius_m=1.05):
    poly=np.asarray(opening['polygon_map_m'],float)
    if poly.ndim!=2 or poly.shape[1]!=2 or len(poly)<3 or not np.isfinite(poly).all():
        raise ValueError('invalid_container_opening')
    if not math.isfinite(handoff_radius_m) or handoff_radius_m<1.05:raise ValueError('invalid_handoff_radius')
    center=poly.mean(axis=0);table=selector.tables[furniture_id] if target_bounds is None else target_bounds
    # This is only the navigation-to-manipulation handoff range. The release
    # adapter measures palm/object geometry and approaches the mouth afterwards.
    cells=[];positions={};a=selector.origin[2];c,s=math.cos(a),math.sin(a)
    lo=selector.cell(center);n=math.ceil(handoff_radius_m/selector.res)+2
    for y in range(lo[0]-n,lo[0]+n+1):
        for x in range(lo[1]-n,lo[1]+n+1):
            cell=(y,x)
            if not selector.valid(cell) or not selector.walk[cell]:continue
            local=np.array([(x+.5)*selector.res,(y+.5)*selector.res])
            xy=np.array([c*local[0]-s*local[1],s*local[0]+c*local[1]])+selector.origin[:2]
            if not .55<=np.linalg.norm(xy-center)<=handoff_radius_m or not inside(*xy,room_polygon):continue
            if not selector.clear_view(xy,table):continue
            cells.append(cell);positions[cell]=xy
    distances=selector.lengths(selector.cell(pose),cells)
    if not distances:raise ValueError('no_reachable_container_approach_region')
    # Navigate into the region, not to its outer boundary: Nav2 may stop
    # 8.5 cm early. Handoff still accepts every clear approach side.
    interior={k:d for k,d in distances.items() if np.linalg.norm(positions[k]-center)<=handoff_radius_m-.20}
    current=selector.cell(pose)
    if current in distances:interior[current]=0.
    if not interior:raise ValueError('no_reachable_container_approach_interior')
    best=min(interior,key=interior.get);xy=positions[best]
    region=dict(mode='container_approach',polygon=room_polygon,cells=[list(k) for k in distances],
                resolution=selector.res,origin=list(selector.origin),opening_center=center.tolist(),handoff_radius_m=handoff_radius_m)
    return [float(xy[0]),float(xy[1]),math.atan2(center[1]-xy[1],center[0]-xy[0])],region

def container_handoff_radius(opening,table,carry_radius_m):
    """Transfer control before the carried envelope reaches the target wall.

    This is a navigation handoff, not a claim the arm can reach from this pose.
    The placement adapter raises above the rim then measures its own approach.
    """
    if not math.isfinite(carry_radius_m) or carry_radius_m<=0:raise ValueError('invalid_carry_radius')
    b=table['axis_aligned_bounds'];c=b['center'];size=b['size']
    center=np.asarray(opening['polygon_map_m'],float).mean(0)
    corners=np.array([[c['x']+sx*size['x']/2,c['y']+sy*size['y']/2] for sx in (-1,1) for sy in (-1,1)])
    target_extent=float(np.linalg.norm(corners-center,axis=1).max())
    return max(1.05,carry_radius_m+target_extent+.20)

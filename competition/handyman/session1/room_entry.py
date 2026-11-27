"""Scope navigation completion to room membership, not a furniture goal."""
from contextlib import contextmanager
from pathlib import Path
import json
import math
from perception_bridge import inside

def entered(pose,region):
    if region.get('mode')=='container_approach':
        x,y=pose[0]-region['origin'][0],pose[1]-region['origin'][1];a=region['origin'][2];r=region['resolution']
        cell=[math.floor((-math.sin(a)*x+math.cos(a)*y)/r),math.floor((math.cos(a)*x+math.sin(a)*y)/r)]
        return inside(float(pose[0]),float(pose[1]),region['polygon']) and cell in region['cells']
    if region.get('mode')=='position_only':
        return inside(float(pose[0]),float(pose[1]),region['polygon']) and math.dist(pose[:2],region['goal'][:2])<=region.get('xy_tolerance_m',.085)
    if region.get('mode')=='grasp_staging':
        from grasp_staging import at_goal
        return inside(float(pose[0]),float(pose[1]),region['polygon']) and at_goal(pose,region['goal'],xy_tolerance=.12,yaw_tolerance=math.radians(10))
    if region.get('mode')=='near_furniture':
        x,y=map(float,pose[:2]);b=region['bounds'];c=b['center'];size=b['size']
        dx=max(abs(x-c['x'])-size['x']/2,0.)
        dy=max(abs(y-c['y'])-size['y']/2,0.)
        return inside(x,y,region['polygon']) and math.hypot(dx,dy)<=region['distance_m']
    polygon=region['polygon'];x,y=map(float,pose[:2])
    margin=float(region.get('entry_margin_m',.5))
    if not math.isfinite(margin) or margin<0:raise ValueError('invalid_room_entry_margin')
    if not inside(x,y,polygon):return False
    # Distance to every finite boundary segment also handles concave rooms.
    distances=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1];length2=dx*dx+dy*dy
        t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/length2)) if length2 else 0.
        distances.append(math.hypot(x-a[0]-t*dx,y-a[1]-t*dy))
    return bool(distances) and min(distances)+1e-9>=margin

def completion_basis(pose,region,*,final_goal_reached=False):
    if entered(pose,region):
        if region.get('mode')=='container_approach':return 'reachable_container_approach_any_heading'
        if region.get('mode')=='position_only':return 'position_reached_orientation_unconstrained'
        if region.get('mode')=='grasp_staging':return 'grasp_staging_within_12cm_10deg'
        return 'near_furniture_current_view' if region.get('mode')=='near_furniture' else 'current_base_center_inside_room_with_0_5m_margin'
    if (region.get('mode')=='near_furniture' and final_goal_reached
            and inside(float(pose[0]),float(pose[1]),region['polygon'])):
        return 'furniture_navigation_goal_reached_first'
    return None

@contextmanager
def finish_on_room_entry(motion,output,room,polygon,*,furniture=None,distance_m=1.0,staging_goal=None,position_goal=None):
    path=Path(output)/('room-entry-'+room+'.json')
    spec=dict(room=room,polygon=polygon,entry_margin_m=.5)
    if furniture is not None:
        path=Path(output)/'furniture-observation-arrival.json'
        spec.update(mode='near_furniture',bounds=furniture['axis_aligned_bounds'],distance_m=distance_m)
    if staging_goal is not None:
        if furniture is not None:raise ValueError('conflicting_navigation_completion_modes')
        path=Path(output)/'grasp-staging-arrival.json'
        spec.update(mode='grasp_staging',goal=list(staging_goal),xy_tolerance_m=.12,yaw_tolerance_rad=math.radians(10))
    if position_goal is not None:
        path=Path(output)/'position-only-arrival.json'
        spec.update(mode='position_only',goal=list(position_goal),xy_tolerance_m=.085)
    path.write_text(json.dumps(spec))
    previous=getattr(motion,'command',None)
    if previous is None:
        # Pure test doubles do not launch child navigation processes.
        yield
        return
    def command(args,*a,**kw):
        if Path(str(args[0])).name=='navigate_carry_trial.py':
            args=[Path(__file__).with_name('room_entry_navigation.py'),*args[1:],'--room-region',path]
        return previous(args,*a,**kw)
    motion.command=command
    try:yield
    finally:motion.command=previous

@contextmanager
def finish_on_container_approach(motion,output,room,region):
    path=Path(output)/'container-approach-region.json'
    path.write_text(json.dumps(dict(region,room=room)))
    previous=getattr(motion,'command',None)
    if previous is None:
        yield
        return
    def command(args,*a,**kw):
        if Path(str(args[0])).name=='navigate_carry_trial.py':
            args=[Path(__file__).with_name('room_entry_navigation.py'),*args[1:],'--room-region',path]
        return previous(args,*a,**kw)
    motion.command=command
    try:yield
    finally:motion.command=previous

"""Rank already geometry-checked recovery poses; RGB is advisory, never depth."""
import math
import numpy as np

def angle_distance(a,b):
    return abs((a-b+math.pi/2)%math.pi-math.pi/2)

def hand_support(point_world,hint):
    if hint is None:return 0.
    p=(np.asarray(point_world)-hint['camera_world'])@hint['rotation_world']
    raster=p@hint['raster_to_frame']
    if raster[2]<=.02:return 0.
    uv=hint['k']@raster;u,v=np.rint(uv[:2]/uv[2]).astype(int)
    image=hint['rgb'];h,w=image.shape[:2]
    if not (3<=u<w-3 and 3<=v<h-3):return 0.
    patch=image[v-3:v+4,u-3:u+4].reshape(-1,3).astype(float)
    colours=np.asarray(hint['reference_rgb'],float)[::max(1,len(hint['reference_rgb'])//100)]
    if len(colours)==0:return 0.
    errors=np.linalg.norm(patch[:,None]-colours[None],axis=2).min(axis=1)
    return float(np.mean(errors<55))

def choose(decision,failed,world_point=lambda p:p,hint=None):
    candidates=decision.get('candidates',[])
    if not candidates or failed is None:return decision
    strategy=decision['selected']['strategy'] if decision.get('selected') else None
    pool=[c for c in candidates if c['strategy']==strategy]
    if not pool:return decision
    rows=[]
    for c in pool:
        angle=angle_distance(c['wrist_roll_rad'],failed['wrist_roll_rad'])
        visual=hand_support(world_point(c['grasp_point_base_m']),hint)
        rows.append((c,angle,visual))
    rotated=[r for r in rows if r[1]>=math.radians(20)]
    choices=rotated or rows
    # Valid material overlap and full hand sweep already checked by planner.
    # Prefer useful narrow contact, not arbitrarily the thinnest/noisiest point.
    best=max(choices,key=lambda r:(r[2]>=.5, r[0]['score']+.2*r[2]-.8*abs(r[0]['observed_width_m']-.035)))
    result=dict(decision,selected=best[0],recovery_selection=dict(
        method='new_wrist_angle_and_local_measured_span',angle_change_rad=best[1],
        hand_rgb_support=best[2],hand_rgb_available=hint is not None,
        hand_rgb_depth_measured=False,used_alternative_angle=bool(rotated)))
    return result

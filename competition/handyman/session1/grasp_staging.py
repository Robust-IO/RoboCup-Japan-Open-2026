"""Furniture-facing base staging, generated only after measured target identity."""
import math
import numpy as np
from perception_bridge import inside
from grasp_runtime.autograsp.kinematics import forward,posture,CONTACT

NAVIGATION_GAPS_M=(1.0,1.15,1.30)

def candidates(table,target_map,room_polygon,*,base_radius=None,audit=None,angle_offsets=(0.,),gaps=NAVIGATION_GAPS_M):
    b=table.get('oriented_bounds') or table['axis_aligned_bounds']
    c=np.array([b['center']['x'],b['center']['y']],float)
    half=np.array([b['size']['x'],b['size']['y']],float)/2
    a=float(b.get('yaw',0));r=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    target=np.asarray(target_map,float)
    if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('measured_target_xyz_required')
    palm,rotation=forward(posture('side'));lateral=float((palm+rotation@CONTACT)[1])
    result=[]
    for axis in (0,1):
        for sign in (-1,1):
            normal=r[:,axis]*sign
            for offset in angle_offsets:
                if not math.isfinite(offset) or abs(offset)>math.radians(15)+1e-9:
                    raise ValueError('grasp_approach_angle_outside_15_degrees')
                co,si=math.cos(offset),math.sin(offset)
                outward=np.array([[co,-si],[si,co]])@normal
                heading=-outward;left=np.array([-heading[1],heading[0]])
                if base_radius is not None:
                    reaches=[]
                    for strategy in ('side','overhead'):
                        palm0,rot0=forward(posture(strategy))
                        reaches.append(float((palm0+rot0@CONTACT)[0]))
                    reach=max(reaches)
                    inset=float(half[axis]-np.dot(target[:2]-c,normal))
                    # Conservative extra reach for an oblique approach and the
                    # calibrated lateral gripper offset. Preserve reach filtering.
                    required=max(0.,inset)/co+float(base_radius)+abs(lateral*math.tan(offset))
                    reachable=required<=reach
                    if audit is not None:audit.append(dict(axis=axis,sign=sign,approach_offset_rad=offset,
                        target_inset_m=inset,base_radius_m=float(base_radius),
                        required_reach_m=required,max_calibrated_reach_m=reach,
                        accepted=reachable,reason=None if reachable else 'target_too_deep_from_this_side'))
                    if not reachable:continue
                aligned=target[:2]-left*lateral
                for gap in gaps:
                    distance=(half[axis]+gap-np.dot(aligned-c,normal))/co
                    xy=aligned+outward*distance
                    yaw=math.atan2(heading[1],heading[0])
                    if inside(*xy,room_polygon):result.append([float(xy[0]),float(xy[1]),yaw])
    return result

def at_goal(pose,goal,*,xy_tolerance=.08,yaw_tolerance=.08):
    yaw=math.atan2(math.sin(pose[2]-goal[2]),math.cos(pose[2]-goal[2]))
    return math.dist(pose[:2],goal[:2])<=xy_tolerance and abs(yaw)<=yaw_tolerance


def staging_policy(config):
    flexible=True
    return dict(flexible=flexible,
        angle_offsets=tuple(math.radians(a) for a in (0,-10,10,-15,15)) if flexible else (0.,),
        xy_tolerance=.12 if flexible else .08,
        yaw_tolerance=math.radians(10) if flexible else .08)

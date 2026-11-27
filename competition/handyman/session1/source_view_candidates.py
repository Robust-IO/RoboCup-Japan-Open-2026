"""Close furniture-front observation poses, independent of hidden object positions."""
import math
import numpy as np

def near_front_candidates(table,room_polygon,standoff=.55):
    from perception_bridge import inside
    b=table.get('oriented_bounds') or table['axis_aligned_bounds'];c=b['center'];size=b['size']
    center=np.array([c['x'],c['y']],float);half=np.array([size['x'],size['y']],float)/2
    a=float(b.get('yaw',0.));r=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    front=float(table['front_yaw']);candidates=[]
    # A corner seat's transform-forward can face a wall or adjacent furniture.
    # Enumerate nearby side views too; the existing map/visibility selector decides.
    for offset in (0.,math.pi/4,-math.pi/4,math.pi/2,-math.pi/2,math.pi):
        angle=front+offset;u=np.array([math.cos(angle),math.sin(angle)]);side=np.array([-u[1],u[0]])
        extent=float(np.abs(r.T@u)@half);side_extent=float(np.abs(r.T@side)@half)
        for gap in (standoff,standoff+.15,standoff+.3):
            for shift in (0.,-.5,.5):
                p=center+u*(extent+gap)+side*side_extent*shift
                if inside(*p,room_polygon):candidates.append([float(p[0]),float(p[1]),math.atan2(center[1]-p[1],center[0]-p[0])])
    return candidates


def source_candidates(config,environment,table):
    """Single source of observation poses for fixed and dynamically bound tasks."""
    from execution_plan import furniture_candidates
    mode=config.get('session_profile',{}).get('source_observation',{}).get('mode')
    if mode=='near_furniture_front':
        poses=near_front_candidates(table,environment['rooms'][table['room']]['region'])
        if not poses:raise ValueError('source_front_has_no_in_room_candidate')
        return poses
    return furniture_candidates(environment,table['id'])

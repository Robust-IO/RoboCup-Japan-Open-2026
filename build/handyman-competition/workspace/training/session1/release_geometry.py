"""User-selected release gate: gripping center over the table at release height."""
import math
import numpy as np

def release_check(center,vertices,plane,table,plan,height_range):
    center=np.asarray(center,float);vertices=np.asarray(vertices,float)
    coef=np.asarray(plane['plane_map'],float)
    if center.shape!=(3,) or vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or coef.shape!=(3,) or not np.isfinite(np.r_[center,vertices.ravel(),coef]).all():
        raise ValueError('finite_release_geometry_required')
    if plane['destination_id']!=table['id'] or plan['destination_id']!=table['id']:
        raise ValueError('wrong_release_destination')
    bounds=table.get('oriented_bounds') or table['axis_aligned_bounds']
    xy=bounds['center'];size=bounds['size'];yaw=bounds.get('yaw',0.)
    c,s=math.cos(yaw),math.sin(yaw)
    relative=np.array([[c,s],[-s,c]])@(center[:2]-[xy['x'],xy['y']])
    inside=bool((abs(relative)<=np.array([size['x'],size['y']])/2).all())
    gap=float(np.min(vertices[:,2]-np.c_[vertices[:,:2],np.ones(len(vertices))]@coef))
    height_ok=height_range[0]<=gap<=height_range[1]
    return dict(ready=bool(inside and height_ok),gripper_center_map_m=center.tolist(),
                gripper_center_inside_destination=inside,object_support_disk_check_enabled=False,bottom_gap_m=gap,height_ok=bool(height_ok),
                basis='current_gripping_center_over_destination_and_measured_release_height')

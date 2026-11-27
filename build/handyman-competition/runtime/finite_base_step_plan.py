"""Finite odom-frame proposal for the inspected HSR Unity implementation.

Pure planning, no publisher. Bounds assume stationary base before reception,
the inspected source, and unchanged coordinate hierarchy; not collision proof.
"""
import math


def remaining_translation(odom_pose, target_odom):
    """Fixed endpoint expressed at the scan-time base pose; never a new step."""
    if len(odom_pose)!=3 or len(target_odom)!=3 or not all(math.isfinite(v) for v in (*odom_pose,*target_odom)):
        raise ValueError('invalid_input')
    x,y,yaw=odom_pose;dx=target_odom[0]-x;dy=target_odom[1]-y
    c,s=math.cos(yaw),math.sin(yaw)
    return [c*dx+s*dy,-s*dx+c*dy]


def plan(odom_pose, direction=(.09336279355,-.0299586638), distance=.015, duration=2., simulation_large_step=False, simulation_contact_trial=False):
    if len(odom_pose)!=3 or len(direction)!=2 or not all(math.isfinite(x) for x in (*odom_pose,*direction,distance,duration)):
        raise ValueError('invalid_input')
    limit=(.25 if direction[0]<0 else .1) if simulation_contact_trial else (.035 if simulation_large_step else .015)
    if not 0<distance<=limit or not 2<=duration<=3:raise ValueError('outside_reviewed_step_limits')
    length=math.hypot(*direction)
    if length==0:raise ValueError('zero_direction')
    dx,dy=(x*distance/length for x in direction);x,y,yaw=odom_pose
    if not -math.pi<=yaw<=math.pi:raise ValueError('yaw_not_wrapped')
    # Avoid the source receiver's Euler-angle wrap boundary on the first trial.
    if abs(yaw)>math.pi-.05:raise ValueError('yaw_wrap_boundary')
    c,s=math.cos(yaw),math.sin(yaw)
    target=[x+c*dx-s*dy,y+s*dx+c*dy,yaw]
    bound=distance+math.sqrt(2)*.0015
    return dict(motion_authorized=False,frame='odom',joint_names=['odom_x','odom_y','odom_t'],
        positions=target,time_from_start_s=duration,relative_base_translation_m=[dx,dy],
        nominal_distance_m=distance,stationary_source_based_distance_bound_m=bound,
        nominal_speed_mps=distance/duration,simulation_large_step=simulation_large_step,simulation_contact_trial=simulation_contact_trial,
        caveats=['Bound assumes the inspected source, fresh TF, and stationary start through command receipt.',
                 'Duration uses Unity Time.time; pause suspends progress and resume may continue it.',
                 'Twist zero is rejected while omni trajectory is running; it is not cancellation.',
                 'Endpoint may be undershot when FixedUpdate skips past the final duration.',
                 'Only one segment may be sent; fresh final-state confirmation is required before another.'])

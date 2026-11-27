"""Use available lift travel; retain a real minimum lift and reclose check."""
def lift_goal(joints,policy):
    goal=list(joints)
    minimum=min(.05,policy.lift_check_m)
    if minimum<.02:raise ValueError('verification_lift_minimum_too_small')
    distance=min(.08,policy.max_lift_m-goal[0])
    if distance+1e-6<minimum:raise ValueError('verification_lift_headroom_insufficient')
    goal[0]+=distance
    return goal,distance

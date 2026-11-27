"""Select coherent original stamps; never retimestamp or widen consumer gates.

Legacy callers retain the fixed TF delay. Live capture can instead supply a
nonblocking image-time TF predicate and reserve time for saving/inference.
"""
def select_stamps(history,now_ns,last=None,*,ready=None,joint_stamp_ns=None,
                  processing_reserve_ns=0,joint_processing_reserve_ns=None,joint_max_age_ns=400_000_000):
    if (type(now_ns) is not int or type(processing_reserve_ns) is not int
            or not 0<=processing_reserve_ns<400_000_000):
        raise ValueError('invalid_capture_time_budget')
    joint_reserve=processing_reserve_ns if joint_processing_reserve_ns is None else joint_processing_reserve_ns
    if type(joint_reserve) is not int or not 0<=joint_reserve<400_000_000:
        raise ValueError('invalid_joint_time_budget')
    if type(joint_max_age_ns) is not int or joint_max_age_ns not in (400_000_000,600_000_000):
        raise ValueError('invalid_joint_age_limit')
    if joint_stamp_ns is not None:
        if type(joint_stamp_ns) is not int or not -20_000_000<=now_ns-joint_stamp_ns<=joint_max_age_ns-joint_reserve:
            return None
    for depth in sorted(history['depth'],reverse=True):
        if ready is None:
            if not 100_000_000<=now_ns-depth<=450_000_000:continue
        elif not 0<=now_ns-depth<=400_000_000-processing_reserve_ns:continue
        rgb=min(history['rgb'],key=lambda s:abs(s-depth),default=None)
        hand=min((s for s in history['hand'] if depth<=s<=depth+200_000_000),default=None)
        if rgb is None or hand is None or abs(rgb-depth)>30_000_000:continue
        selected=(rgb,depth,hand)
        if any(not 0<=now_ns-s<=500_000_000 for s in selected):continue
        if last is not None and any(s<=old for s,old in zip(selected,last)):continue
        if ready is not None:
            if any(now_ns-s>400_000_000-processing_reserve_ns for s in selected):continue
            if not ready(selected):continue
        return selected
    return None

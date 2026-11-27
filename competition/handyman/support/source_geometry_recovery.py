"""Reobserve missing 3D information; physical gripper limits remain authoritative."""
DATA_FAILURES=('insufficient','not_observed','not_above_table','detection_clipped',
    'no_dominant','noisy','surface_missing','colour_not_distinct','not_seen')

def acquire_measured_source(acquire,solve,aim,record):
    last=None
    # Three independent frames per view; bounded diagnostic storage, no blind base move.
    for attempt in range(9):
        view=attempt//3
        if attempt in (3,6):aim((0.,(-.5,-.6,-.4)[view]))
        sample,detections=acquire(attempt)
        try:return sample,detections,solve(sample,detections)
        except ValueError as exc:
            if not any(reason in str(exc) for reason in DATA_FAILURES):raise
            last=exc;record(dict(event='source_geometry_reobserve',attempt=attempt+1,view=view,reason=str(exc),base_commands_sent=False))
    raise ValueError('source_geometry_unresolved_after_three_views:'+str(last))

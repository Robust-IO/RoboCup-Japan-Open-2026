"""Optional pregrasp lift/orient combination; no retries after motion errors."""
import time
import numpy as np
from .coordinator import AttemptFailure
from .recovery import Failure

def prepare_arm(b,raised,goal):
    started=time.monotonic()
    b._sample_for_path()
    initial=np.asarray(b.rt.state()['joints'],float)
    goal=np.asarray(goal,float)
    count=max(2,int(np.max(abs(goal[:5]-initial[:5]))/.04)+1)
    fallback=None
    try:
        b._check_open_path(np.linspace(initial,goal,count))
    except AttemptFailure as exc:
        if exc.failure not in (Failure.APPROACH_COLLISION,Failure.OCCLUDED):raise
        fallback=str(exc)
    if fallback is None:
        # _arm validates again, then publishes once. Execution exceptions must
        # propagate; they must never cause a second blind movement.
        b._arm(goal,'generic-combined-raise-orient')
        mode='combined'
    else:
        b.record(dict(event='combined_pregrasp_fallback',reason=fallback,motion_sent=False))
        b._arm(raised,'generic-raise-before-orient')
        b._sample_for_path()
        b._arm(goal,'generic-orient-open')
        mode='sequential'
    b.record(dict(event='pregrasp_arm_preparation_complete',mode=mode,
                  elapsed_s=time.monotonic()-started,planned_motion_count=1 if mode=='combined' else 2))

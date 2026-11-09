"""Wait for new valid feedback only after cancellation of a navigation goal.

No thresholds change, no gripper/arm action and no goal dispatch occur here.
The caller may resume only the unfinished waypoint after this function returns.
"""
import math
import time
from navigation_feedback_gate import NavigationFeedbackError


def recover_stopped_feedback(check,pump,cancel,*,on_event=lambda _:None,clock=time.monotonic,budget_s=5.):
    cancel()  # Must confirm goal termination before any recovery wait.
    on_event(dict(event='navigation_cancel_confirmed_before_feedback_wait'))
    started=clock();deadline=started+budget_s;first=None
    while clock()<deadline:
        try:sample=check()
        except NavigationFeedbackError as exc:
            if type(exc.age_s) not in (int,float) or not math.isfinite(exc.age_s) or exc.age_s<0:raise
        else:
            if clock()>=deadline:break
            stamps=sample['stamps_ns']
            if first is None:first=stamps
            elif all(stamps[k]>first[k] for k in ('scan','tf')):
                on_event(dict(event='stopped_feedback_recovered',waited_s=clock()-started,
                    first_stamps_ns=first,last_stamps_ns=stamps,original_age_limits_unchanged=True))
                return sample
        pump(min(.02,max(0.,deadline-clock())))
    raise RuntimeError('stopped_navigation_feedback_recovery_expired')

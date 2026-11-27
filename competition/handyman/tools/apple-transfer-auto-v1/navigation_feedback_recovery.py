"""Wait for new valid feedback only after cancellation of a navigation goal.

No thresholds change, no gripper/arm action and no goal dispatch occur here.
The caller may resume only the unfinished waypoint after this function returns.
"""
import math
import time
from navigation_feedback_gate import NavigationFeedbackError


RECOVERY_LIMIT=10

def recovery_window(previous_count):
    if type(previous_count) is not int or previous_count<0:raise ValueError('invalid_navigation_recovery_count')
    return min(previous_count+1,RECOVERY_LIMIT), None if previous_count>=RECOVERY_LIMIT else 5.


def recover_stopped_feedback(check,pump,cancel,*,on_event=lambda _:None,clock=time.monotonic,budget_s=5.):
    cancel()  # Must confirm goal termination before any recovery wait.
    on_event(dict(event='navigation_cancel_confirmed_before_feedback_wait'))
    started=clock();deadline=None if budget_s is None else started+budget_s;first=None;notice=started
    while deadline is None or clock()<deadline:
        try:sample=check()
        except NavigationFeedbackError as exc:
            if type(exc.age_s) not in (int,float) or not math.isfinite(exc.age_s) or exc.age_s<0:raise
        else:
            if deadline is not None and clock()>=deadline:break
            stamps=sample['stamps_ns']
            if first is None:first=stamps
            elif all(stamps[k]>first[k] for k in ('scan','tf')):
                on_event(dict(event='stopped_feedback_recovered',waited_s=clock()-started,
                    first_stamps_ns=first,last_stamps_ns=stamps,original_age_limits_unchanged=True))
                return sample
        if clock()-notice>=5:
            on_event(dict(event='waiting_for_navigation_feedback',elapsed_s=clock()-started,task_retained=True));notice=clock()
        pump(.02 if deadline is None else min(.02,max(0.,deadline-clock())))
    raise RuntimeError('stopped_navigation_feedback_recovery_expired')

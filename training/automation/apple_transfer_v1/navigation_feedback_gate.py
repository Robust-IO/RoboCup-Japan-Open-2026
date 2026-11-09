"""Original sample-age limits plus bounded NEW-feedback acquisition. No ROS/control.

The existing Nav2 goal is not paused by this read-only acquisition helper. A
transient age crossing may therefore defer cancellation by AT MOST150ms; this
is an explicit supervision-policy change, not a claim that the robot stops
while waiting. No stale sample authorizes a new goal or successful endpoint.
"""
import math
import time

MAX_SAMPLE_AGE_S = .7
REACQUIRE_BUDGET_S = .15


class NavigationFeedbackError(RuntimeError):
    def __init__(self, stream, age_s):
        self.stream, self.age_s = stream, age_s
        super().__init__('stale_navigation_feedback:' + stream)


def validate_ages(ages,*,joint_max_age_s=MAX_SAMPLE_AGE_S):
    if joint_max_age_s not in (.7,.85):raise ValueError('unsupported_navigation_joint_age')
    # Invalid/future or already>850ms input must not hide behind a different
    # stream's small age overrun and accidentally receive a grace window.
    for key in ('joints', 'scan', 'tf'):
        value = ages.get(key)
        limit=joint_max_age_s if key=='joints' else MAX_SAMPLE_AGE_S
        if (type(value) not in (int,float) or not math.isfinite(value)
                or value < -.05 or value > limit+REACQUIRE_BUDGET_S):
            raise NavigationFeedbackError(key, value)
    for key in ('joints','scan','tf'):
        if ages[key] > (joint_max_age_s if key=='joints' else MAX_SAMPLE_AGE_S):raise NavigationFeedbackError(key,ages[key])


def acquire_fresh_feedback(check, pump, *, on_event=lambda event: None, clock=time.monotonic,
        joint_max_age_s=MAX_SAMPLE_AGE_S):
    """Re-read feedback only, within one fixed150ms deadline; never replay actions.

    Missing/invalid/future data and holding/control failures fail immediately.
    A stream already older than850ms is not granted another150ms window.
    Late processing cannot turn expiry into a successful authorization.
    """
    if joint_max_age_s not in (.7,.85):raise ValueError('unsupported_navigation_joint_age')
    started = deadline = last_fault = None
    while True:
        if deadline is not None and clock() >= deadline:
            on_event(dict(event='expired', waited_s=clock()-started, reason=str(last_fault)))
            raise last_fault
        try:
            value = check()
        except NavigationFeedbackError as error:
            age = error.age_s
            limit=joint_max_age_s if error.stream=='joints' else MAX_SAMPLE_AGE_S
            if type(age) not in (int,float) or not math.isfinite(age) or not limit < age <= limit+REACQUIRE_BUDGET_S:
                raise
            last_fault = error
            if started is None:
                started = clock();deadline = started+REACQUIRE_BUDGET_S
                on_event(dict(event='waiting_for_new_sample', reason=str(error), age_s=age,
                    budget_s=REACQUIRE_BUDGET_S, existing_goal_paused=False))
            remaining = deadline-clock()
            if remaining > 0:pump(min(.01,remaining))
        else:
            if deadline is not None:
                if clock() >= deadline:
                    on_event(dict(event='expired', waited_s=clock()-started, reason=str(last_fault)))
                    raise last_fault
                on_event(dict(event='new_fresh_sample_received', waited_s=clock()-started))
            return value

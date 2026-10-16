"""Explicit session budget, not a short per-navigation test deadline."""
import math


def session_budget(value):
    if type(value) not in (int,float) or not math.isfinite(value) or not 3<=value<=600:
        raise ValueError('session budget must be actual remaining seconds in [3,600]')
    return float(value)


def remaining(deadline,now):
    value=deadline-now
    if not math.isfinite(value) or value<=0:raise ValueError('session_budget_expired')
    return value


def stops_session(event):
    # The moderator repeats readiness questions while waiting for a response.
    # They are not evidence that the current session ended.
    return event in ('Task_failed','Task_succeeded','Mission_complete')


class SessionEvents:
    """Initial/repeated environment announcements are not session boundaries."""
    def __init__(self):
        self.environment=None

    def stops(self,event,detail=''):
        if event=='Environment':
            value=detail.strip()
            if not value:return True
            if self.environment is None:
                self.environment=value
                return False
            return value!=self.environment
        return stops_session(event)

"""Cancellable recovery at a repeatable checkpoint, never whole-task replay."""
import time

def check_cancel(cancel):
    if cancel is not None and cancel.is_set():raise RuntimeError('task_cancelled')

def transient(exc):
    name=type(exc).__name__;reason=str(exc).lower()
    if 'cancel' in reason or 'interrupted' in reason:return False
    if name in ('LookupException','ConnectivityException','ExtrapolationException'):return True
    if name=='AttemptFailure':
        return str(getattr(exc,'failure','')).lower().split('.')[-1] in ('sensor_unavailable','occluded','no_candidate')
    return any(token in reason for token in ('stale','unsynchron','clock_changed','not_new','new_destination_depth_frame','new_hold_observation','missing_navigation_feedback','missing_navigation_tf','fresh_measurement','no_valid_depth','insufficient_depth'))

def retry_read(call,*,cancel=None,record=None,repair=None,retry_occluded=True):
    attempt=0
    while True:
        check_cancel(cancel)
        try:return call()
        except Exception as exc:
            if not transient(exc):raise
            if not retry_occluded and str(getattr(exc,'failure','')).lower().split('.')[-1] in ('occluded','no_candidate'):raise
            attempt+=1
            if record:record(dict(event='checkpoint_recovery',attempt=attempt,reason=str(exc),action='fresh_observation',task_retained=True))
            if repair:repair(attempt)
            if cancel is not None:
                if cancel.wait(min(.2*attempt,1.)):check_cancel(cancel)
            else:time.sleep(min(.2*attempt,1.))

def motion_cancel(motion):return getattr(motion,'session_cancel_event',None)

from contextlib import contextmanager

@contextmanager
def grasp_observation_recovery(backend,runtime,cancel):
    """Cover every grasp observation, including reset code outside attempt try."""
    original=backend.observe_stopped
    def observe(*args,**kwargs):
        return retry_read(lambda:original(*args,**kwargs),cancel=cancel,
            record=getattr(backend,'record',None),
            repair=lambda attempt:runtime.face_source((0.,(-.3,-.5,-.7,-.8,0.)[(attempt-1)%5])))
    backend.observe_stopped=observe
    try:yield
    finally:backend.observe_stopped=original

"""Bounded wait for fresh feedback; never publishes or relaxes validity limits."""
import time

RETRYABLE = frozenset(('stale_joints', 'stale_capture', 'stale_base_tf'))


def wait_for_fresh(check, pump, on_wait=lambda: None, *, timeout=1.0,
                   clock=time.monotonic):
    deadline = clock() + timeout
    waiting = False
    while True:
        try:
            result = check()
        except RuntimeError as error:
            if str(error) not in RETRYABLE:
                raise
            if not waiting:
                waiting = True
                on_wait()
            if clock() >= deadline:
                raise RuntimeError('feedback_wait_expired:' + str(error)) from error
            pump(min(.02, max(0., deadline - clock())))
        else:
            # A pump/check that consumed the remaining budget cannot authorize motion.
            if waiting and clock() >= deadline:
                raise RuntimeError('feedback_wait_expired')
            return result

"""Non-blocking, bounded retries of one exact image-time TF lookup."""
import math

class ExactStampWait:
    def __init__(self, stamp_ns, started, timeout_s=.5):
        if stamp_ns<=0 or not math.isfinite(started) or not 0<timeout_s<=2:
            raise ValueError('invalid_tf_wait')
        self.stamp_ns=stamp_ns
        self.started=started
        self.deadline=started+timeout_s
        self.error=None

    def poll(self, lookup, now, fresh=True):
        # Caller services ROS callbacks between polls; never sleep inside here.
        if not fresh:
            return 'expired',None,self.error
        if now>=self.deadline or now<self.started:
            return 'timeout',None,self.error
        try:
            return 'ready',lookup(self.stamp_ns),None
        except Exception as exc:
            self.error=str(exc)
            return 'waiting',None,self.error

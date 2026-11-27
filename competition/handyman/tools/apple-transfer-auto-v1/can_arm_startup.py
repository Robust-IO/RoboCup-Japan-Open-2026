"""Fresh observed transform readiness despite asynchronous joint/TF arrivals.

This is a startup-only latch, never a motion lease or substitute joint state.
The caller still checks latest feedback, ownership and actual pose before motion.
"""


class CurrentTransformReadiness:
    def __init__(self):
        self.stamp_ns=None

    def ready(self,latest_ns,now_ns,available):
        def fresh(stamp):
            return type(stamp) is int and stamp>0 and -.02e9<=now_ns-stamp<=.4e9
        if type(now_ns) is not int or not fresh(latest_ns):
            self.stamp_ns=None
            return False
        # Recheck the actual selected transform, not just a remembered boolean.
        # Newer joints can arrive before their TF without invalidating this
        # still-fresh observed sample during the150ms startup stability window.
        if fresh(self.stamp_ns) and available(self.stamp_ns):return True
        self.stamp_ns=None
        if available(latest_ns):
            self.stamp_ns=latest_ns
            return True
        return False

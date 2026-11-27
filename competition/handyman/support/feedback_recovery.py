"""Observe-only recovery policy for an already finite, non-repeated segment.

Recovery validates current feedback, never fills in unobserved trajectory data.
"""
class Recovery:
    transient=frozenset(('stale_capture','no_fresh_aligned_scan','tf_not_ready'))

    def __init__(self):
        self.fault=None;self.gap_start=None;self.valid_since=None
        self.last_good=None;self.first_stamp=None;self.last_stamp=None;self.gaps=[]

    def bad(self,now,reason):
        self.valid_since=None;self.first_stamp=None;self.last_good=None
        if reason not in self.transient:self.fault=self.fault or reason
        if self.gap_start is None:self.gap_start=now
        if now-self.gap_start>=1.:self.fault=self.fault or 'feedback_gap_exceeded_1s'

    def good(self,now,stamp_ns):
        if self.gap_start is not None:
            duration=now-self.gap_start
            self.gaps.append(dict(duration_s=duration,recovered=True))
            if duration>=1.:self.fault=self.fault or 'feedback_gap_exceeded_1s'
            self.gap_start=None
        if self.last_stamp is not None and stamp_ns<self.last_stamp:
            self.bad(now,'feedback_clock_reversed');return
        if self.last_stamp==stamp_ns:return
        if self.last_good is None or now-self.last_good>.4 or (stamp_ns-self.last_stamp)>400_000_000:
            self.valid_since=now;self.first_stamp=stamp_ns
        self.last_good=now;self.last_stamp=stamp_ns

    def verified(self,now):
        return (self.fault is None and self.gap_start is None and self.valid_since is not None
                and now-self.last_good<=.4 and now-self.valid_since>=.6
                and self.last_stamp-self.first_stamp>=600_000_000)

    def summary(self,now):
        return dict(current_feedback_verified=self.verified(now),hard_fault=self.fault,
                    recovered_gaps=self.gaps,unresolved_gap=self.gap_start is not None,
                    unobserved_interval_remains_unverified=bool(self.gaps or self.gap_start is not None),
                    automatic_next_segment=False)

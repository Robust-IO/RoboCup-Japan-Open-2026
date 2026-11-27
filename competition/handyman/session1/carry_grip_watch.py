"""Passive joint anomaly detector. Unchanged aperture does not prove retention."""
import math
class GripWatch:
    def __init__(self,reference):
        self.reference=reference;self.last=None;self.since=None;self.count=0
    def observe(self,opening,stamp_ns,age_s):
        if opening is None or not math.isfinite(opening) or not 0<=age_s<=.85:
            self.since=None;self.count=0;return False
        if self.last is not None and stamp_ns<=self.last:return False
        self.last=stamp_ns
        suspect=(opening<=.01 or self.reference-opening>.04 or opening-self.reference>.08)
        if not suspect:self.since=None;self.count=0;return False
        if self.since is None:self.since=stamp_ns
        self.count+=1
        return self.count>=3 and stamp_ns-self.since>=300_000_000

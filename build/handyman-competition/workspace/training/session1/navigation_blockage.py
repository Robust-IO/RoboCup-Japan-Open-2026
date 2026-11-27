"""Escalate sustained command vetoes; temporary guests may clear naturally."""
import math
class BlockageWatch:
    def __init__(self,wait_s=4.):
        self.wait_s=wait_s;self.since=None;self.anchor=None
    def update(self,now,pose,blocked):
        if not blocked:
            self.since=None;self.anchor=None;return False
        if self.since is None or math.dist(pose[:2],self.anchor[:2])>.03:
            self.since=now;self.anchor=list(pose);return False
        return now-self.since>=self.wait_s

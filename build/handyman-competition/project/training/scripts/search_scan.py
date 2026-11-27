"""Offline search scheduler. Emits descriptions only, never ROS commands.

Search-point completion is NOT proof of room coverage or Does_not_exist.
The adapter must verify navigation arrival/settling and RGBD health independently.
"""
import math

class SearchScan:
    def __init__(self,task_id,target,points,started,timeout_s=60.,view_s=5.,negative_frames=3):
        if not task_id or not target or not points or negative_frames<3:
            raise ValueError('missing_search_configuration')
        if not all(math.isfinite(x) for x in (started,timeout_s,view_s)) or min(timeout_s,view_s)<=0:
            raise ValueError('invalid_timing')
        self.points=[]
        for p in points:
            pose={k:float(p[k]) for k in ('x','y','yaw')}
            if not all(math.isfinite(v) for v in pose.values()): raise ValueError('invalid_point')
            self.points.append(pose)
        self.task_id=task_id; self.target=target; self.deadline=started+timeout_s
        self.view_s=view_s; self.required=negative_frames; self.index=0
        self.phase='navigate'; self.view_start=None; self.last_stamp=0; self.negative=0
        self.completed=[]; self.failures=[]; self.position=None
        self.evidence_ready=None  # Optional stationary-view evidence predicate.
        self.supplement_s=3.0

    def status(self):
        return dict(task_id=self.task_id,target=self.target,state=self.phase,
                    completed_views=list(self.completed),failures=list(self.failures),
                    does_not_exist_authorized=False,actionable=False)

    def request(self):
        if self.phase not in ('navigate','observe'): return self.status()
        return dict(self.status(),view_id=f'{self.task_id}:{self.index}',
                    request=self.phase,pose=dict(self.points[self.index]),frame_id='map')

    def tick(self,now):
        if not math.isfinite(now): raise ValueError('invalid_time')
        if self.phase not in ('navigate','observe'): return self.status()
        if now>=self.deadline:
            self.phase='incomplete'; self.failures.append('search_timeout')
        elif self.phase=='observe' and now-self.view_start>=self.view_s:
            # Bounded supplemental capture; never renew the session deadline.
            if (self.evidence_ready is None or self.evidence_ready(now) or
                    now-self.view_start>=self.view_s+self.supplement_s):
                self.advance('observation_timeout')
        return self.status()

    def advance(self,failure=None):
        if failure: self.failures.append(f'{self.index}:{failure}')
        else: self.completed.append(self.index)
        self.index+=1; self.negative=0; self.last_stamp=0
        self.phase=('incomplete' if self.failures else 'plan_exhausted') if self.index==len(self.points) else 'navigate'

    def arrived(self,view_id,success,settled,now):
        self.tick(now)
        if self.phase!='navigate' or view_id!=f'{self.task_id}:{self.index}': return self.status()
        if not success: self.advance('navigation_failed')
        elif settled:
            self.phase='observe'; self.view_start=now; self.last_stamp=0; self.negative=0
        return self.status()

    def observe(self,view_id,target,stamp_ns,now,healthy,stable=False,position=None,absent=False):
        self.tick(now)
        if self.phase!='observe' or view_id!=f'{self.task_id}:{self.index}' or target!=self.target:
            return self.status()
        # now is monotonic; stamp_ns is the sensor clock. Freshness must be
        # checked by the adapter and included in healthy, never compare clocks.
        if not healthy or stamp_ns<=0 or stamp_ns<=self.last_stamp:
            self.negative=0; return self.status()
        self.last_stamp=stamp_ns
        if stable and not absent:
            if position is None or len(position)!=3 or not all(math.isfinite(v) for v in position):
                self.negative=0; return self.status()
            self.position=tuple(position); self.phase='found'; return self.status()
        if absent and not stable:
            self.negative+=1
            if self.negative>=self.required: self.advance()
        else: self.negative=0
        return self.status()

    def cancel(self):
        self.phase='cancelled'; self.position=None; self.negative=0
        return self.status()

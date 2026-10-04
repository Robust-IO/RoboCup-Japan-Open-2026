"""Offline recovery-policy prototype. NOT wired to the live runtime.

Inputs must be independently verified by a future owner. Proposals authorize
neither movement nor target/absence answers. Deadline uses one monotonic clock.
"""
import math


class SearchRecoveryPlan:
    def __init__(self, task, map_digest, point_ids, deadline):
        if not task or not map_digest or not point_ids or len(set(point_ids))!=len(point_ids):
            raise ValueError('invalid_identity')
        if any(not isinstance(p,str) or not p for p in point_ids):
            raise ValueError('invalid_points')
        if not math.isfinite(deadline):raise ValueError('invalid_deadline')
        self.task=task;self.map_digest=map_digest;self.points=tuple(point_ids)
        self.deadline=deadline;self.index=0;self.completed=[];self.pending=[]
        self.retries={};self.waiting=False;self.state='ready';self.last_now=None

    def _clock(self,now):
        if not math.isfinite(now) or (self.last_now is not None and now<self.last_now):
            raise ValueError('invalid_clock')
        self.last_now=now
        if now>=self.deadline and self.state not in ('cancelled','blocked','exhausted'):
            self.state='expired'

    def status(self):
        return dict(state=self.state,point_id=self.points[self.index] if self.index<len(self.points) else None,
                    completed_points=list(self.completed),pending_review=[dict(r) for r in self.pending],
                    deadline=self.deadline,actionable=False,does_not_exist_authorized=False,
                    live_integration=False)

    def propose(self,now,*,sensors_ready=False,stationary=False):
        self._clock(now)
        if self.state in ('ready','waiting'):
            if self.waiting and (sensors_ready is not True or stationary is not True):
                self.state='waiting'
            else:self.state='ready'
        result=self.status()
        result['proposal']='observe_current_point' if self.state=='ready' else 'none'
        return result

    def record(self,*,task,map_digest,point_id,outcome,now,cleanup_verified,
               observation_verified=False):
        if (task,map_digest)!=(self.task,self.map_digest):raise ValueError('identity_mismatch')
        self._clock(now)
        if self.state not in ('ready','waiting'):return self.status()
        if point_id!=self.points[self.index]:raise ValueError('out_of_order_point')
        if outcome not in ('observed_empty','weak_candidate','transient_motion','fatal'):
            raise ValueError('unsupported_outcome')
        if cleanup_verified is not True:
            self.state='blocked';return self.status()
        if outcome=='fatal':
            self.state='blocked';return self.status()
        if outcome=='transient_motion':
            attempt=self.retries.get(point_id,0)
            if attempt>=1:self.state='blocked'
            else:
                self.retries[point_id]=attempt+1;self.waiting=True;self.state='waiting'
            return self.status()
        if self.state!='ready' or observation_verified is not True:
            self.state='blocked';return self.status()
        # Weak evidence is deferred, never reclassified as negative evidence.
        if outcome=='weak_candidate':
            self.pending.append(dict(point_id=point_id,reason='weak_candidate',resolved=False))
        else:self.completed.append(point_id)
        self.index+=1;self.waiting=False
        self.state='ready' if self.index<len(self.points) else 'exhausted'
        return self.status()

    def cancel(self):
        self.state='cancelled';return self.status()

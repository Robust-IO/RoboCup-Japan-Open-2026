"""Non-actuating multi-point policy. Inputs must be verified by its future owner.

Does not itself verify arrival, map, cleanup or camera coverage. No room absence
authorization, even after every point. Used by the runtime's domain-73-only mode.
"""
import math


class SearchPointSequence:
    def __init__(self,task,target,digest,point_ids):
        if not all(isinstance(v,str) and v for v in (task,target,digest)):
            raise ValueError('missing_identity')
        if not point_ids or any(not isinstance(v,str) or not v for v in point_ids) or len(set(point_ids))!=len(point_ids):
            raise ValueError('invalid_points')
        self.task,self.target,self.digest=task,target,digest
        self.points=list(point_ids);self.index=0;self.state='awaiting_point'
        self.visited=[];self.position=None;self.reason=None

    def status(self):
        return dict(task_id=self.task,target=self.target,state=self.state,
                    point_id=self.points[self.index] if self.index<len(self.points) else None,
                    visited_points=list(self.visited),position_m=self.position,reason=self.reason,
                    actionable=False,does_not_exist_authorized=False)

    def cancel(self):
        self.state='cancelled';self.position=None;self.reason='task_cancelled'
        return self.status()

    def complete_observation(self,row):
        evidence=row.get('view_evidence',{}) if isinstance(row,dict) else {}
        if not isinstance(evidence,dict) or evidence.get('schema')!='handyman-view-evidence-v1':
            evidence={}
        stamps=evidence.get('continuation_stamps',evidence.get('negative_stamps',[]))
        if 'continuation_stamps' in evidence and evidence.get('continuation_scope')!='view_only_not_room_absence':
            stamps=[]
        arrival=evidence.get('arrival_stamp_ns')
        after_arrival=(type(arrival) is int and arrival>0 and isinstance(stamps,list) and
                       all(type(v) is int and v>arrival for v in stamps))
        return self.complete(row,arrival_verified=evidence.get('arrival_verified') is True and type(arrival) is int and arrival>0,
            negative_stamps=stamps if after_arrival else [],view_data_usable=evidence.get('view_data_usable') is True)

    def complete(self,row,*,arrival_verified,negative_stamps,view_data_usable):
        if self.state!='awaiting_point':return self.status()
        context=dict(task_id=self.task,point_id=self.points[self.index],map_sha256=self.digest)
        if not isinstance(row,dict) or row.get('observer_context')!=context or row.get('task_id')!=self.task or row.get('target')!=self.target:
            raise ValueError('point_identity_mismatch')
        # Never treat a transport event or observer exit as proof of cleanup.
        if row.get('cleanup_verified') is not True:return self.status()
        if row.get('actionable') is not False or row.get('does_not_exist_authorized') is not False:
            raise ValueError('unexpected_authority')
        if row.get('terminal') is not True or arrival_verified is not True:
            self.state='incomplete';self.reason='arrival_or_terminal_unverified';return self.status()
        if row.get('state')=='found':
            position=row.get('position_m')
            if (row.get('position_frame')!='odom' or not isinstance(position,list) or len(position)!=3 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in position)):
                raise ValueError('invalid_position')
            self.position=list(position);self.state='found';return self.status()
        stamps=negative_stamps
        negatives=(isinstance(stamps,list) and len(stamps)>=3 and
                   all(type(v) is int and v>0 for v in stamps) and
                   all(b>a for a,b in zip(stamps,stamps[1:])))
        failures=row.get('failures')
        # Only an observation deadline, not search/navigation/TF timeouts.
        if (row.get('state')!='incomplete' or failures!=['0:observation_timeout'] or
            view_data_usable is not True or not negatives):
            self.state='incomplete';self.reason='point_not_eligible_for_continuation';return self.status()
        self.visited.append(self.points[self.index]);self.index+=1
        self.reason='view_not_found_not_room_absence'
        if self.index==len(self.points):self.state='incomplete';self.reason='points_exhausted_coverage_unverified'
        return self.status()

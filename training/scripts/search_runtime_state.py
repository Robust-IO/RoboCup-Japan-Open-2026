"""No ROS: strict, task-bound observation records for runtime lifecycle decisions."""
import json
import math


class ObserverRecords:
    def __init__(self,task,point,digest,target):
        self.context=dict(task_id=task,point_id=point,map_sha256=digest)
        self.target=target;self.ready=False;self.terminal=None;self.offset=0;self.arrived=False

    def accept(self,row):
        if (not isinstance(row,dict) or row.get('observer_context')!=self.context or
            row.get('task_id')!=self.context['task_id'] or row.get('target')!=self.target or
            row.get('actionable') is not False or row.get('does_not_exist_authorized') is not False):
            raise ValueError('observer_identity_or_authority_mismatch')
        if self.terminal is not None:raise ValueError('record_after_terminal')
        if row.get('observer_reason')=='ready_requires_new_active_goal':self.ready=True
        if row.get('arrival_reason')=='arrived_and_settled':self.arrived=True
        elif row.get('state')=='navigate':self.arrived=False
        if row.get('terminal') is not True:return
        if row.get('state') not in ('found','incomplete','cancelled','plan_exhausted'):
            raise ValueError('invalid_observer_terminal')
        if row['state']=='found':
            point=row.get('position_m')
            if (row.get('position_frame')!='odom' or not isinstance(point,list) or len(point)!=3 or
                not all(type(v) in (float,int) and math.isfinite(v) for v in point)):
                raise ValueError('invalid_found_position')
        self.terminal=dict(row)

    def poll(self,path):
        if not path.exists():return
        if path.stat().st_size>16*1024*1024:raise ValueError('observer_log_limit')
        with path.open('rb') as stream:
            stream.seek(self.offset)
            for line in stream:
                if not line.endswith(b'\n'):break
                self.accept(json.loads(line));self.offset+=len(line)

    def finished(self,process):
        return process is not None and process.poll()==0 and self.terminal is not None

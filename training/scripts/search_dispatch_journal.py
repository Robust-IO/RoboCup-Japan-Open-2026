"""Durable pre-dispatch identities. UNKNOWN is never a terminal resolution."""
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
from search_process_watchdog import valid_id


class DispatchJournal:
    def __init__(self,path):
        self.path=Path(path).resolve();self.path.parent.mkdir(parents=True,exist_ok=True)
        self.lock=(self.path.parent/(self.path.name+'.lock')).open('a+b')
        try:fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BaseException:self.lock.close();raise
        try:
            self.db=sqlite3.connect(str(self.path),timeout=1.)
            self.db.execute('PRAGMA journal_mode=DELETE')
            self.db.execute('PRAGMA synchronous=EXTRA')
            if self.db.execute('PRAGMA user_version').fetchone()[0] not in (0,1):
                raise ValueError('unsupported dispatch journal version')
            if self.db.execute('PRAGMA quick_check').fetchone()[0]!='ok':raise ValueError('corrupt journal')
            with self.db:
                self.db.execute('CREATE TABLE IF NOT EXISTS intents (goal TEXT PRIMARY KEY, payload TEXT NOT NULL, checksum TEXT NOT NULL, terminal INTEGER)')
                self.db.execute('CREATE TABLE IF NOT EXISTS active_requests (task TEXT PRIMARY KEY, bundle_sha256 TEXT NOT NULL)')
                self.db.execute('PRAGMA user_version=1')
            directory=os.open(str(self.path.parent),os.O_RDONLY|os.O_DIRECTORY)
            try:os.fsync(directory)
            finally:os.close(directory)
            self.rows()  # Validate existing identities/checksums before any authorization.
        except BaseException:
            if hasattr(self,'db'):self.db.close()
            self.lock.close();raise

    def claim_active_request(self,task,bundle_sha256):
        if not valid_id(task) or not isinstance(bundle_sha256,str) or len(bundle_sha256)!=64 or any(c not in '0123456789abcdef' for c in bundle_sha256):
            raise ValueError('invalid_active_request_claim')
        try:
            with self.db:
                self.db.execute('INSERT INTO active_requests VALUES (?,?)',(task,bundle_sha256))
        except sqlite3.IntegrityError as exc:
            raise ValueError('active_request_previously_consumed') from exc
        # Committed before any navigation dispatch. A failed run still consumes
        # the task: retries require a new task, never deletion of this record.

    @staticmethod
    def payload(row,action):
        if (not isinstance(row,dict) or row.get('schema')!='handyman-owned-goal-v1' or
            not valid_id(row.get('task_id')) or not valid_id(row.get('goal_id')) or
            type(row.get('generation')) is not int or row['generation']<=0 or
            not isinstance(action,str) or not action.startswith('/') or action.endswith('/') or
            row.get('frame_id')!='map' or row.get('role') not in ('route_waypoint','final_search_point') or
            not isinstance(row.get('point_id'),str) or not row['point_id'] or
            not isinstance(row.get('map_sha256'),str) or len(row['map_sha256'])!=64 or
            any(c not in '0123456789abcdef' for c in row['map_sha256'])):
            raise ValueError('invalid_dispatch_identity')
        pose=row.get('pose')
        if not isinstance(pose,dict) or set(pose)!= {'x','y','yaw'} or not all(type(v) in (int,float) and math.isfinite(v) for v in pose.values()):
            raise ValueError('invalid_dispatch_pose')
        fields=('schema','task_id','point_id','map_sha256','goal_id','generation','frame_id','role','pose')
        return json.dumps(dict({k:row[k] for k in fields},action_name=action),sort_keys=True,separators=(',',':'),allow_nan=False)

    def record(self,row,action):
        payload=self.payload(row,action);key=row['goal_id']
        with self.db:
            prior=self.db.execute('SELECT payload FROM intents WHERE goal=?',(key,)).fetchone()
            if prior:
                if prior[0]!=payload:raise ValueError('conflicting_dispatch_identity')
                return False
            self.db.execute('INSERT INTO intents VALUES (?,?,?,NULL)',(key,payload,hashlib.sha256(payload.encode()).hexdigest()))
        return True  # Only after SQLite's FULL+directory sync transaction committed.

    def resolve(self,key,status,*,rejected=False):
        if type(status) is not int or status not in ((0,) if rejected else (4,5,6)):
            raise ValueError('unknown_is_not_terminal')
        with self.db:
            prior=self.db.execute('SELECT terminal FROM intents WHERE goal=?',(key,)).fetchone()
            if prior is None:raise ValueError('unrecorded_goal')
            if prior[0] is not None and prior[0]!=status:raise ValueError('conflicting_terminal')
            self.db.execute('UPDATE intents SET terminal=? WHERE goal=?',(status,key))

    def rows(self,unresolved=False):
        rows=[]
        for key,payload,checksum,status in self.db.execute('SELECT goal,payload,checksum,terminal FROM intents ORDER BY goal'):
            row=json.loads(payload)
            if (hashlib.sha256(payload.encode()).hexdigest()!=checksum or row.get('goal_id')!=key or
                self.payload(row,row.get('action_name'))!=payload or status not in (None,0,4,5,6)):
                raise ValueError('invalid_persisted_dispatch_record')
            if not unresolved or status is None:rows.append(dict(row,terminal=status))
        return rows

    def close(self):
        self.db.close();self.lock.close()


class IntentGoalCanceller:
    """Exact UUID retries cover cancellation before a delayed acceptance.

    UNKNOWN / rejection of cancel is inconclusive, not success. Never re-send
    SendGoal, cancel-all, erase records, or unlock the coordinator.
    """
    def __init__(self,node,journal,row,timeout=30.):
        import time
        from action_msgs.srv import CancelGoal
        from nav2_msgs.action import NavigateToPose
        self.node,self.journal,self.row=node,journal,row
        self.key=row['goal_id'];self.deadline=time.monotonic()+timeout
        if not valid_id(self.key):raise ValueError('invalid goal')
        self.cancel_type=CancelGoal;self.result_type=NavigateToPose.Impl.GetResultService
        self.cancel_client=node.create_client(CancelGoal,row['action_name']+'/_action/cancel_goal')
        self.result_client=node.create_client(self.result_type,row['action_name']+'/_action/get_result')
        self.cancel_pending=False;self.result_pending=False;self.finished=False;self.events=[]
        self.timer=node.create_timer(.2,self.tick)

    def tick(self):
        import time
        if self.finished:return
        if time.monotonic()>=self.deadline:
            self.finished=True;self.events.append(dict(state='unresolved_intent',goal_id=self.key,task_drained=False));return
        if not self.cancel_pending and self.cancel_client.service_is_ready():
            request=self.cancel_type.Request();request.goal_info.goal_id.uuid=list(bytes.fromhex(self.key))
            self.cancel_pending=True
            def cancelled(future):
                self.cancel_pending=False
                try:future.result()
                except Exception:pass
            self.cancel_client.call_async(request).add_done_callback(cancelled)
        if not self.result_pending and self.result_client.service_is_ready():
            request=self.result_type.Request();request.goal_id.uuid=list(bytes.fromhex(self.key))
            self.result_pending=True
            def result(future):
                self.result_pending=False
                try:
                    status=future.result().status
                    if status in (4,5,6):
                        self.journal.resolve(self.key,status);self.finished=True
                        self.events.append(dict(state='persisted_terminal',goal_id=self.key,status=status,task_drained=False))
                except Exception as exc:self.events.append(dict(state='recovery_error',reason=str(exc),task_drained=False))
            self.result_client.call_async(request).add_done_callback(result)

    def close(self):
        self.node.destroy_timer(self.timer)
        self.node.destroy_client(self.cancel_client);self.node.destroy_client(self.result_client)

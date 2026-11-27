"""Fail-closed process ownership monitor; no ROS, signals, or navigation commands.

Use one monitor per owned worker/task. Process exit (including code zero) is not
proof that its Nav2 goals ended. The owner must keep monitoring until its explicit
task-retirement protocol finishes; this helper never certifies successful drain.
"""
import math
import re
import time


def valid_id(value):
    return isinstance(value, str) and re.fullmatch(r'[0-9a-f]{32}', value) is not None and value != '0'*32


class SearchProcessWatchdog:
    def __init__(self, task_id, process, lifetime_sec, *, clock=time.monotonic):
        if not valid_id(task_id):
            raise ValueError('invalid task_id')
        if isinstance(lifetime_sec, bool) or not math.isfinite(lifetime_sec) or lifetime_sec <= 0:
            raise ValueError('lifetime_sec must be positive and finite')
        self.task_id = task_id
        self.process = process
        self.clock = clock
        self.deadline = clock() + lifetime_sec
        self.fault = None
        self.cancel_id = None
        self.retirement_deadline = None
        self.retired = False

    def sample(self):
        if self.fault is None:
            code = self.process.poll()
            if (self.retirement_deadline is not None and code==0 and
                    (self.retired or self.clock()<self.retirement_deadline)):
                self.retired=True
            elif code is not None:
                self.fault = dict(reason='worker_exited_without_retirement', returncode=code)
            elif self.clock() >= (self.retirement_deadline or self.deadline):
                self.fault = dict(reason='worker_lifetime_exceeded', returncode=None)
        return None if self.fault is None else dict(self.fault)

    def prepare_retirement(self, row):
        """Caller must independently verify all registered goals ended first."""
        if (not isinstance(row,dict) or row.get('schema')!='handyman-search-cancel-status-v1' or
            row.get('task_id')!=self.task_id or self.cancel_id is None or
            row.get('cancel_id')!=self.cancel_id or row.get('state')!='cancel_drained'):
            return False
        if self.sample() is not None:return False
        if self.retirement_deadline is not None:return True
        if self.process.poll() is not None:return False
        self.retirement_deadline=min(self.deadline,self.clock()+2.)
        return True

    def cancellation(self, row):
        if not isinstance(row, dict) or row.get('schema') != 'handyman-search-request-v1':
            return False
        if row.get('task_id') != self.task_id or not valid_id(row.get('cancel_id')):
            return False
        if self.cancel_id is not None and row['cancel_id'] != self.cancel_id:
            return False
        self.cancel_id = row['cancel_id']
        return True

    def failure_report(self):
        fault = self.sample()
        if fault is None or self.cancel_id is None:
            return None
        return dict(schema='handyman-search-cancel-status-v1', task_id=self.task_id,
                    cancel_id=self.cancel_id, state='cancel_failed', **fault)

    def fault_notice(self):
        fault = self.sample()
        if fault is None:
            return None
        return dict(schema='handyman-search-worker-fault-v1', task_id=self.task_id, **fault)

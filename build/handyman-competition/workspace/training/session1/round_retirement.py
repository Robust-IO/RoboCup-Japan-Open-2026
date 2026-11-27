"""Bounded retirement of one owned round and its descendants, never other rounds."""
import os,signal,time
from pathlib import Path

def identities():
    rows={}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            s=(p/'stat').read_text().split(') ',1)[1].split()
            rows[int(p.name)]=(int(s[1]),s[19],s[0])
        except (OSError,IndexError,ValueError):continue
    return rows

class RoundRetirement:
    def __init__(self,pid,record,clock=time.monotonic):
        self.pid=pid;self.record=record;self.clock=clock;self.started=clock();self.owned={};self.level=0
        self.refresh();self.send(signal.SIGINT)
    def refresh(self):
        rows=identities();parents={self.pid,*self.owned}
        changed=True
        while changed:
            changed=False
            for pid,(ppid,birth,state) in rows.items():
                if pid==self.pid or ppid in parents:
                    if pid not in self.owned:self.owned[pid]=birth;parents.add(pid);changed=True
        return {pid for pid,birth in self.owned.items() if pid in rows and rows[pid][1]==birth and rows[pid][2] not in ('Z','X')}
    def send(self,sig):
        # Individual PIDs with start-time identity avoid signalling reused PIDs
        # or unrelated processes sharing a terminal process group.
        for pid in sorted(self.refresh(),key=lambda p:p==self.pid):
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
    def poll(self):
        elapsed=self.clock()-self.started
        if elapsed>=15 and self.level<1:
            self.send(signal.SIGTERM);self.level=1
            self.record(event='retiring_old_task_escalation',pid=self.pid,signal='SIGTERM',elapsed_s=elapsed)
        if elapsed>=20:
            self.send(signal.SIGKILL)
            if self.level<2:self.record(event='retiring_old_task_escalation',pid=self.pid,signal='SIGKILL',elapsed_s=elapsed)
            self.level=2
        return not self.refresh()

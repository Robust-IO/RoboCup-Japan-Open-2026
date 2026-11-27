"""Six-round transition policy; fresh wire handshakes own transitions."""
class RoundSequence:
    layouts=('LayoutA','LayoutB','LayoutA','LayoutC','LayoutD','LayoutD')
    def __init__(self):
        self.index=0;self.started=False;self.done=False;self.pending=None
        self.environment=None;self.environment_at=None;self.ready_at=None;self.finished=False
        self.round_engaged=False
    def message(self,name,detail,now):
        if name=='Mission_complete':self.finished=True;self.pending=None;return
        if name in ('Instruction','Task_succeeded','Task_failed') and self.started:
            # Discard startup readiness; a new handshake must follow actual task
            # participation (or an official terminal event), especially for D->D.
            self.round_engaged=True
            self.ready_at=None;self.environment_at=None
        if name=='Task_failed' and self.started:
            self.done=True
            if self.index==len(self.layouts)-1:self.finished=True
            return
        if name=='Are_you_ready?':self.ready_at=now
        if name=='Environment':self.environment=detail;self.environment_at=now
        if self.finished or self.ready_at is None or self.environment_at is None:return
        if now-self.ready_at>3 or now-self.environment_at>3:return
        if not self.started:
            if self.environment==self.layouts[self.index]:self.pending=self.index
        elif (self.index+1<len(self.layouts) and self.environment==self.layouts[self.index+1]
              and (self.layouts[self.index+1]!=self.layouts[self.index] or self.round_engaged)):
            self.pending=self.index+1
    def launch(self,index):
        expected=self.index+1 if self.started else self.index
        if index!=expected:raise ValueError('rounds_must_start_in_sequence')
        self.index=index;self.started=True;self.done=False;self.pending=None;self.round_engaged=False
        self.ready_at=None;self.environment_at=None;self.environment=None
    def exited(self):
        self.done=True
        if self.index==len(self.layouts)-1:self.finished=True

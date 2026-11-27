"""Nonblocking effect dispatcher. Ports own ROS and all physical verification.

start/poll/cancel must be nonblocking. start must clean partial actions before
raising. poll returns None while active, a terminal dictionary after retirement.
Protocol status=sent means publication, never a Moderator scoring acknowledgement.
"""
from collections import deque

class Executor:
    def __init__(self,session,operations,protocol):
        missing=set(session.steps)-set(operations)
        if missing: raise ValueError('missing_operation_ports:'+','.join(sorted(missing)))
        self.session=session;self.operations=operations;self.protocol=protocol
        self.queue=deque();self.active=None;self.cancelling=False;self.fault=None

    def moderator(self,message,detail='',*,session_id):
        self.session.moderator(message,detail,session_id=session_id)

    def fail(self,reason):
        self.fault=reason;self.session.stop(reason)

    def tick(self):
        effects=self.session.drain()
        if any(e['kind']=='stop' for e in effects):
            self.queue.clear()
            if self.active and not self.cancelling:
                port,handle,effect=self.active
                self.cancelling=True
                try:port.cancel(handle,preserve_grip=True)
                except Exception as exc:self.fault='cancel_failed:'+str(exc)
        else:self.queue.extend(effects)
        if self.active:
            port,handle,effect=self.active
            try:result=port.poll(handle)
            except Exception as exc:
                self.fail('poll_failed:'+str(exc));return
            if result is None:return
            if not isinstance(result,dict):self.fail('invalid_port_result');return
            if result.get('status')=='running':return
            self.active=None
            if self.cancelling:self.cancelling=False;return
            if effect['kind']=='protocol':
                if result.get('status')!='sent':self.fail('protocol_publish_failed')
            else:
                if (result.get('status')!='ok' and result.get('reason') ==
                        'DroppedTargetNotFound:floor_target_not_found_after_retreat'):
                    self.session.give_up(result['reason'])
                else:
                    self.session.result(effect['operation_id'],result,session_id=effect['session_id'],run_id=effect['run_id'])
            return
        if self.session.state in self.session.TERMINAL:self.queue.clear();return
        if not self.queue:return
        effect=self.queue.popleft()
        if effect['kind']=='operation':
            if effect['operation_id']!=self.session.pending:return
            port=self.operations[effect['stage']]
        elif effect['kind']=='protocol':port=self.protocol
        else:self.fail('unsupported_effect');return
        try:self.active=(port,port.start(effect),effect)
        except Exception as exc:self.fail('start_failed:'+str(exc))

    @property
    def retired(self):return self.active is None and not self.queue and not self.session.effects

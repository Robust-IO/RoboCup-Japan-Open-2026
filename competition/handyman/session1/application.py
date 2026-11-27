"""Assemble Session1 with a single action worker and a responsive event loop.

Site adapters supply all stage handlers and a nonblocking stop callback. A handler
must check cancel_event between commands; request_stop must cancel current ROS
motion without opening the hand. Worker retirement alone is not motion stopping.
"""
from concurrent.futures import ThreadPoolExecutor
from queue import SimpleQueue, Empty
from threading import Event
from session1 import Session1
from executor import Executor


class WorkerPort:
    def __init__(self,pool,handler,request_stop):
        self.pool=pool;self.handler=handler;self.request_stop=request_stop

    def start(self,effect):
        cancelled=Event()
        def run():
            if cancelled.is_set():return {'status':'cancelled'}
            try:return self.handler(effect,cancelled)
            except Exception as exc:return {'status':'failed','reason':type(exc).__name__+':'+str(exc)}
        return {'future':self.pool.submit(run),'cancel':cancelled,'stop_future':None,'stop_required':False,'effect':effect}

    def poll(self,handle):
        future=handle['future'];stop=handle['stop_future']
        if handle['stop_required'] and stop is None:raise RuntimeError('motion_stop_confirmation_missing')
        if not future.done() or (stop is not None and not stop.done()):return None
        result=future.result()
        if not isinstance(result,dict):
            result={'status':'failed','reason':'invalid_stage_result'}
        if result.get('status')!='ok' and not handle['stop_required']:
            # An action can fail after publishing a finite trajectory. Do not
            # retire its ownership merely because its Python handler returned.
            import json
            print(json.dumps(dict(event='stage_failed_before_stop_check',stage=handle['effect']['stage'],result=result)),flush=True)
            self.cancel(handle,preserve_grip=True)
            return None
        if stop is not None:
            proof=stop.result()
            ended=(isinstance(proof,dict) and proof.get('session_ended') is True
                   and proof.get('commands_retired') is True and proof.get('official_end') is not None)
            if not isinstance(proof,dict) or (proof.get('motion_stopped') is not True and not ended):
                raise RuntimeError('motion_stop_not_confirmed')
        return result

    def cancel(self,handle,*,preserve_grip):
        handle['cancel'].set()
        handle['stop_required']=True
        # Adapter returns a Future; control-loop callback must never block.
        handle['stop_future']=self.request_stop(handle['effect'],preserve_grip=preserve_grip)
        if handle['stop_future'] is None:raise RuntimeError('stop_confirmation_future_required')


class ProtocolPort:
    def __init__(self,publish):self.publish=publish
    def start(self,effect):return self.publish(effect)
    def poll(self,future):
        if future.cancelled():return {'status':'cancelled'}
        return future.result() if future.done() else None
    def cancel(self,future,*,preserve_grip):
        # A message already sent cannot be unsent; never retry on cancellation.
        future.cancel()


class Application:
    def __init__(self,session_id,handlers,publish,request_stop,*,profile=None):
        if profile and profile.get('schema')=='handyman_dynamic_instructions_v1':
            from dynamic_session import DynamicSession
            self.session=DynamicSession(session_id,profile=profile)
        elif profile and profile.get('schema')=='handyman_session3_search_v1':
            from session3 import Session3
            self.session=Session3(session_id,profile=profile)
        else:self.session=Session1(session_id,source_table='white_side_table#living_room',profile=profile)
        missing=set(self.session.steps)-set(handlers)
        if missing:raise ValueError('missing_live_handlers:'+','.join(sorted(missing)))
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='session1-action')
        ports={stage:WorkerPort(self.pool,handlers[stage],request_stop) for stage in self.session.steps}
        self.executor=Executor(self.session,ports,ProtocolPort(publish))
        self.messages=SimpleQueue()

    def receive(self,message,detail='',*,session_id):
        self.messages.put((message,detail,session_id))

    def tick(self):
        while True:
            try:message,detail,session_id=self.messages.get_nowait()
            except Empty:break
            self.executor.moderator(message,detail,session_id=session_id)
        self.executor.tick()

    def close(self):
        if not self.executor.retired:
            raise RuntimeError('cancel_and_wait_for_motion_retirement_before_close')
        self.pool.shutdown(wait=True)

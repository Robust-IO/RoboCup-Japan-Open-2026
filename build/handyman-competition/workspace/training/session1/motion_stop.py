"""Cooperative cancellation plus measured retirement, no emergency grip release.

Runtime publishers are gated before publishing; already sent finite trajectories
finish. Only the owned navigation child receives SIGINT (its existing cleanup
cancels Nav2). Verification runs on the same single ROS action thread afterward.
"""
from pathlib import Path
from concurrent.futures import Future,ThreadPoolExecutor
from threading import Event,Lock
import math
import os
import signal
import time
import json

class MotionGate:
    def __init__(self,clock=time.monotonic):
        self.cancelled=Event();self.clock=clock;self.deadline=0.;self.lock=Lock();self.child_command_uncertain=False
    def check(self):
        if self.cancelled.is_set():raise RuntimeError('task_cancelled_no_new_motion')
    def publish(self,publish,message):
        with self.lock:
            self.check()
            duration=max([float(p.time_from_start.sec)+p.time_from_start.nanosec/1e9 for p in getattr(message,'points',[])]+[0.])
            self.deadline=max(self.deadline,self.clock()+duration)
            return publish(message)
    def cancel(self):
        with self.lock:self.cancelled.set()
    def install(self,runtime,motion):
        # Patch the actual ROS publisher's publish method, retaining the original
        # publisher object required by node.destroy_publisher().
        motion.session_motion_gate=self
        motion.session_cancel_event=self.cancelled
        original=runtime.publisher
        def publisher(*a,**kw):
            pub=original(*a,**kw)
            if not getattr(pub,'_session1_gated',False):
                publish=pub.publish
                pub.publish=lambda message:self.publish(publish,message)
                pub._session1_gated=True
            return pub
        runtime.publisher=publisher
        command=motion.command
        def guarded_command(*a,**kw):
            self.check();self.child_command_uncertain=True
            self.navigation_report_path=None
            args=list(map(str,a[0])) if a else []
            if any(Path(arg).name in ('navigate_carry_trial.py','carry_navigation_runtime.py','room_entry_navigation.py','efficient_navigation.py') for arg in args) and '--output' in args:
                self.navigation_report_path=Path(args[args.index('--output')+1])
            result=command(*a,**kw)
            self.child_command_uncertain=False
            return result
        motion.command=guarded_command


def stationary_proof(states):
    if len(states)<3:raise ValueError('three_fresh_stop_samples_required')
    stamps=[s['stamp_ns'] for s in states]
    if any(b<=a for a,b in zip(stamps,stamps[1:])) or stamps[-1]-stamps[0]<300_000_000:
        raise ValueError('distinct_stop_feedback_required')
    anchor=states[0]
    for state in states:
        j=state['joints'];p=state['poses']['odom'];a=anchor['poses']['odom']
        if len(j)!=6 or len(p)!=3 or not all(math.isfinite(v) for v in [*j,*p]):raise ValueError('invalid_stop_feedback')
        yaw=math.atan2(math.sin(p[2]-a[2]),math.cos(p[2]-a[2]))
        if max(abs(x-y) for x,y in zip(j,anchor['joints']))>.01 or math.hypot(p[0]-a[0],p[1]-a[1])>.005 or abs(yaw)>.01:
            raise ValueError('motion_still_observed')
    return dict(motion_stopped=True,feedback_stamps_ns=stamps,grip_release_command_sent=False)

class StopController:
    def __init__(self,runtime,motion,gate,output):
        self.runtime=runtime;self.motion=motion;self.gate=gate;self.output=output
        self.application=None;self.supervisor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='session1-stop')
        self.future=None
    def bind(self,application):self.application=application
    def request_stop(self,effect,*,preserve_grip):
        if preserve_grip is not True:raise ValueError('stop_must_preserve_grip')
        if self.application is None:raise ValueError('stop_controller_not_bound')
        self.gate.cancel()
        if self.future is not None:return self.future
        active=self.application.executor.active
        action_future=active[1]['future'] if active and active[2]['kind']=='operation' else None
        self.future=self.supervisor.submit(self._retire,action_future,effect)
        return self.future
    def _retire(self,action_future,effect):
        signalled=set()
        while action_future is not None and not action_future.done():
            child=getattr(self.motion,'active_child',None)
            is_navigation=child is not None and any(os.path.basename(str(arg)) in ('navigate_carry_trial.py','carry_navigation_runtime.py','room_entry_navigation.py') for arg in getattr(child,'args',[]))
            if is_navigation and child.poll() is None and child.pid not in signalled:
                args=list(map(str,getattr(child,'args',[])))
                if '--output' in args:self.navigation_report_path=Path(args[args.index('--output')+1])
                try:os.killpg(child.pid,signal.SIGINT);signalled.add(child.pid)
                except ProcessLookupError:pass
            time.sleep(.02)
        # All runtime sampling stays on the action thread. No concurrent spin on
        # runtime.node, and no release of ownership while confirmation is pending.
        attempt=0
        while True:
            try:return self.application.pool.submit(self._verify,effect,bool(signalled)).result()
            except (RuntimeError,ValueError) as exc:
                from task_recovery import transient
                if str(exc) not in ('fresh_stationary_stop_not_confirmed','motion_still_observed','finite_child_still_running','navigation_child_still_running','navigation_stop_report_unavailable','navigation_stop_not_confirmed') and not transient(exc):raise
                attempt+=1
                (self.output/'stop-recovery.json').write_text(json.dumps(dict(status='retrying_stop_feedback',attempt=attempt,reason=str(exc),grip_release_command_sent=False)))
                time.sleep(.2)
    def _verify(self,effect,navigation_interrupted):
        # Official simulation termination is different from measured stopping.
        # Retire only after every child exited and finite command duration elapsed.
        ended=getattr(self.application,'official_simulation_end',None)
        child=getattr(self.motion,'active_child',None)
        if ended and ended.get('session_id')==self.application.session.session_id:
            if child is not None and child.poll() is None:raise RuntimeError('finite_child_still_running')
            while time.monotonic()<self.gate.deadline:self.runtime.pump(.02)
            proof=dict(motion_stopped=False,session_ended=True,commands_retired=True,
                       official_end=ended,grip_release_command_sent=False)
            (self.output/'stop-confirmation.json').write_text(json.dumps(proof))
            return proof
        # Grasp/withdraw can run navigation too. Classify the actual child,
        # not the high-level stage that happened to launch it.
        owned_report=getattr(self.gate,'navigation_report_path',None)
        navigation_confirmed=False
        if navigation_interrupted or (owned_report is not None and self.gate.child_command_uncertain):
            path=(getattr(self.gate,'navigation_report_path',None) or getattr(self,'navigation_report_path',None) or
                  self.output/('session1-'+effect['operation_id'].replace('/','-')+'.json'))
            child=getattr(self.motion,'active_child',None)
            if child is not None and child.poll() is None:
                raise RuntimeError('navigation_child_still_running')
            try:report=json.loads(path.read_text())
            except (FileNotFoundError,json.JSONDecodeError) as exc:
                raise RuntimeError('navigation_stop_report_unavailable') from exc
            if report.get('nav_reset') is not True or report.get('cancel_error') or report.get('nav_reset_error') or report.get('carry_footprint_restored') is False:
                raise RuntimeError('navigation_stop_not_confirmed')
            navigation_confirmed=True
        if self.gate.child_command_uncertain and not navigation_confirmed:
            child=getattr(self.motion,'active_child',None)
            if child is not None and child.poll() is None:
                raise RuntimeError('finite_child_still_running')
            # Endpoint success and retirement are different: an exited finite
            # child may have failed its goal. Fresh stationary feedback below
            # proves only that ownership can retire, never task success.
        while time.monotonic()<self.gate.deadline:self.runtime.pump(.02)
        states=[];deadline=time.monotonic()+6.
        while time.monotonic()<deadline:
            self.runtime.pump(.02)
            # Single read: do not enter the runtime's unlimited stale-data recovery.
            state=(self.runtime._state() if hasattr(self.runtime,'_state') else self.runtime.state())
            if not states or state['stamp_ns']>states[-1]['stamp_ns']:states.append(state)
            if len(states)>=3 and states[-1]['stamp_ns']-states[0]['stamp_ns']>=300_000_000:
                try:
                    proof=stationary_proof(states)
                    (self.output/'stop-confirmation.json').write_text(json.dumps(proof))
                    return proof
                except ValueError:states=states[-1:]
            self.runtime.pump(.16)
        raise RuntimeError('fresh_stationary_stop_not_confirmed')
    def close(self):
        if self.future is not None and not self.future.done():raise RuntimeError('stop_confirmation_pending')
        self.supervisor.shutdown(wait=True)

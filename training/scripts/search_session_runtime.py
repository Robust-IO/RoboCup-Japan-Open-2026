"""Opt-in single-view search lifecycle owner. Default: disabled.

Owns observer + worker, waits for both before forwarding cancel_drained.
Does not emit competition answers, run inference, or certify room-wide absence.
Only a clean session can be replaced; faults latch until operator restart.
"""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import time
import uuid
import yaml

from search_request_consumer import RequestConsumer,RequestMapGate
from search_runtime_state import ObserverRecords
from verify_live_map import decode
from search_timing import session_budget,remaining,SessionEvents


class SearchRuntime:
    def __init__(self,node,args):
        from handyman_msgs.msg import HandymanMsg
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self.node,self.args,self.Msg=node,args,HandymanMsg
        self.active_binding=None
        if getattr(args,'active_bundle',None):
            from active_run_binding import ActiveRunBinding
            self.active_binding=ActiveRunBinding(args.active_bundle,args.package_share)
        # A restarted runtime must never renew an old worker's lease or ACK its
        # pending messages, even if a fresh transient request reuses the task ID.
        self.namespace=args.private_prefix+'/run_'+uuid.uuid4().hex
        from search_dispatch_journal import DispatchJournal,IntentGoalCanceller
        self.journal=DispatchJournal(args.journal)
        pending=self.journal.rows(unresolved=True)
        if not args.allow_live and any(r['action_name']!='/handyman_test/navigate_to_pose' for r in pending):
            self.journal.close();raise ValueError('journal action outside isolated test scope')
        self.recovery=[IntentGoalCanceller(node,self.journal,row,timeout=args.seconds) for row in pending]
        self.recovery_remaining=len(pending)
        self.session_deadline=(time.monotonic()+args.session_budget_seconds
                               if args.session_budget_seconds is not None else None)
        self.consumer=RequestConsumer(args.package_share,args.session_budget_seconds or 30.)
        self.gate=RequestMapGate(self.consumer)
        self.pool=ThreadPoolExecutor(max_workers=1);self.job=None;self.latest=None
        self.session=None;self.locked=bool(pending);self.stopping=False
        self.status=node.create_publisher(HandymanMsg,args.status_topic,10)
        self.results=node.create_publisher(HandymanMsg,args.result_topic,10)
        self.requests=node.create_subscription(HandymanMsg,args.request_topic,self.on_request,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.session_events=None
        self.session_event_state=SessionEvents()
        if self.session_deadline is not None:
            self.session_events=node.create_subscription(HandymanMsg,args.session_event_topic,self.on_session_event,10)
        self.map_sub=node.create_subscription(OccupancyGrid,args.map_topic,self.on_map,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer=node.create_timer(.05,self.tick)
        self.events=(args.output/'events.jsonl').open('x')
        self.log('runtime_ready',execution_enabled=True,namespace=self.namespace,
                 session_budget_seconds=args.session_budget_seconds,
                 short_navigation_deadlines_enabled=self.session_deadline is None)
        if pending:self.log('recovery_locked',goals=[r['goal_id'] for r in pending])

    def on_session_event(self,msg):
        if self.session_event_state.stops(msg.message,msg.detail):
            self.log('session_stop_event',message=msg.message)
            self.fail('session_ended_or_changed')
            self.stopping=True

    def log(self,event,**data):
        self.events.write(json.dumps(dict(event=event,time=time.monotonic(),**data),allow_nan=False)+'\n');self.events.flush()

    def publish(self,publisher,event,row):
        msg=self.Msg();msg.message=event;msg.detail=yaml.safe_dump(row);publisher.publish(msg)

    def publishers(self):
        return sorted(bytes(p.endpoint_gid).hex() for p in self.node.get_publishers_info_by_topic(self.args.map_topic))

    def on_map(self,msg):
        known=self.publishers();self.latest=(msg,known[0] if len(known)==1 else None)
        if self.gate.task_id is not None:self.gate.receive_map(*self.latest,known)

    def on_request(self,msg):
        from search_process_watchdog import valid_id
        row=None
        try:
            row=yaml.safe_load(msg.detail)
            if not isinstance(row,dict):return
            if msg.message=='search_cancelled':
                if not valid_id(row.get('cancel_id')):return
                self.consumer.receive(msg.message,row,self.node.get_clock().now().nanoseconds,time.monotonic())
                s=self.session
                if s is not None and row.get('task_id')==s['task']:
                    if s.get('local_transition'):
                        s['public_cancel']=row;s['candidate']=None
                        s['sequence'].cancel()
                        self.log('external_cancel_during_transition',task_id=s['task'])
                        return
                    if s['cancel'] is None:
                        s['cancel']=row;s['cancel_time']=time.monotonic()
                        if row.get('reason')!='search_stop:observation_terminal':s['candidate']=None
                        self.publish(s['cancel_pub'],'search_cancelled',row)
                        self.log('cancel_forwarded',task_id=s['task'])
                return
            if msg.message!='search_requested':return
            if self.locked or self.stopping:raise ValueError('runtime_locked_or_stopping')
            if self.session is not None:raise ValueError('previous_session_not_retired')
            if self.active_binding:self.active_binding.accept(row,self.journal)
            prepared=self.consumer.receive(msg.message,row,self.node.get_clock().now().nanoseconds,time.monotonic())
            if self.session_deadline is not None:
                remaining(self.session_deadline,time.monotonic())
                self.consumer.expires=self.session_deadline
            point=prepared['points'][self.args.point_index]
            task=prepared['task_id'];directory=self.args.output/task;directory.mkdir()
            prefix=self.namespace+'/task_'+task
            from rclpy.qos import QoSProfile,DurabilityPolicy
            cancel_pub=self.node.create_publisher(self.Msg,prefix+'/request',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            s=dict(task=task,raw=row,prepared=prepared,point=point,directory=directory,prefix=prefix,
                observer=None,worker=None,supervisor=None,manager=None,responder=None,private_sub=None,
                cancel_pub=cancel_pub,cancel=None,cancel_time=None,candidate=None,fault=None,
                started=time.monotonic(),handles=[],records=ObserverRecords(task,point['id'],prepared['map_bundle_sha256'],row['target']))
            s['point_index']=self.args.point_index
            if self.args.multi_point:
                from search_point_sequence import SearchPointSequence
                s['sequence']=SearchPointSequence(task,row['target'],prepared['map_bundle_sha256'],
                    [p['id'] for p in prepared['points'][self.args.point_index:]])
            self.session=s
            self.job=self.pool.submit(self.prepare_execution,s)
            self.log('request_prepared',task_id=task,point_id=point['id'])
        except Exception as exc:
            self.log('request_rejected',reason=str(exc))
            if self.session is not None:self.fail('search_runtime_failed')
            elif isinstance(row,dict) and valid_id(row.get('task_id')):
                self.locked=True
                self.publish(self.status,'search_worker_fault',dict(schema='handyman-search-worker-fault-v1',
                    task_id=row['task_id'],reason='search_runtime_failed'))

    def prepare_execution(self,s):
        if getattr(self.args,'enable_active_ring',False) or getattr(self.args,'check_current_path',False):
            from active_live_entry import plan_current_path
            plan_current_path(self.active_binding,s['directory']/'current-path.json')
        return decode(Path(s['prepared']['map_path']),self.args.decoder)

    def fail(self,reason):
        s=self.session
        self.locked=True
        if s is not None and s['fault'] is None:
            s['fault']=reason;self.log('session_fault',task_id=s['task'],reason=reason)
            # Local fault containment must not depend on a live coordinator.
            # This emergency ID is private to exact registered-goal cancellation;
            # it never authorizes public cancel_drained or task advancement.
            self.stop_lease(s)
            s['emergency_cancel']=dict(schema='handyman-search-request-v1',
                task_id=s['task'],cancel_id=uuid.uuid4().hex,reason='runtime_fault')
            self.publish(s['cancel_pub'],'search_cancelled',s['emergency_cancel'])
            if s['manager'] is not None:
                s['manager'].request_cancel(s['emergency_cancel'])

    def stop_lease(self,s):
        supervisor=s['supervisor']
        subscription=getattr(supervisor,'lease_subscription',None)
        if subscription is not None:
            self.node.destroy_subscription(subscription)
            supervisor.lease_subscription=None

    def start_observer(self,s):
        p=s['point']['pose'];script=Path(__file__).with_name('search_arrival_ros.py')
        observer_seconds=(remaining(self.session_deadline,time.monotonic())
                          if self.session_deadline is not None else self.args.observation_seconds)
        command=[sys.executable,str(script),'--task-id',s['task'],'--point-id',s['point']['id'],
            '--map-sha256',s['prepared']['map_bundle_sha256'],'--binding-topic',s['prefix']+'/binding',
            '--cancel-topic',s['prefix']+'/request','--target',s['raw']['target'],
            '--motion-source','tf','--recover-transient-tf','--action-status-topic',self.args.action_name+'/_action/status',
            '--vision-topic',self.args.vision_topic,'--seconds',str(observer_seconds),
            '--max-bound-attempts','3','--output',str(s['directory']/'observer.jsonl')]
        for key in ('x','y','yaw'):command+=['--'+key,str(p[key])]
        env=yaml.safe_load(Path(s['prepared']['environment_path']).read_text())
        command+=['--target-room',s['raw']['room'],'--room-regions',
                  json.dumps({name:room['region'] for name,room in env['rooms'].items()})]
        if self.args.view_seconds is not None:command+=['--view-seconds',str(self.args.view_seconds)]
        if self.session_deadline is not None:command+=['--session-budget-mode']
        if self.args.head_tilt is not None:
            from search_head_stage import HeadStage
            proof=s['directory']/'head-ready.json'
            s['head']=HeadStage(self.node,s['task'],s['point']['id'],self.args.head_tilt,proof,allow_live=self.args.allow_live)
            command+=['--head-proof',str(proof),'--head-tilt',str(self.args.head_tilt)]
        handle=(s['directory']/'observer.console.log').open('x');s['handles'].append(handle)
        s['observer']=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT)
        self.log('observer_started',task_id=s['task'],pid=s['observer'].pid)

    def observer_done(self,s):
        return s['records'].finished(s['observer'])

    def start_worker(self,s):
        if self.active_binding:self.active_binding.check()
        from owned_goal_registry import OwnedGoalRegistry,RegisteredGoalsCanceller
        from search_map_responder import SearchMapResponder
        from search_process_supervisor import SearchProcessSupervisor
        from search_process_watchdog import SearchProcessWatchdog
        env=yaml.safe_load(Path(s['prepared']['environment_path']).read_text())
        allowed=dict(final_search_point=[s['point']['pose']],route_waypoint=[p for r in env.get('routes',[])
            if r['to']==s['raw']['room'] for p in r['waypoints']])
        registry=OwnedGoalRegistry(s['task'],s['point']['id'],s['prepared']['map_bundle_sha256'],allowed)
        prefix=s['prefix']
        s['manager']=RegisteredGoalsCanceller(self.node,registry,topic=prefix+'/owned_goals',
            ack_topic=prefix+'/owned_goal_ack',action_name=self.args.action_name,enable_live=self.args.allow_live,journal=self.journal)
        s['responder']=SearchMapResponder(self.node,self.consumer,self.gate,self.publishers,
            request_topic=prefix+'/map_check/request',reply_topic=prefix+'/map_check/reply')
        s['private_sub']=self.node.create_subscription(self.Msg,prefix+'/execution_status',
            lambda msg:self.on_worker_status(msg) if self.session is s else None,10)
        worker_lifetime=(min(600.,remaining(self.session_deadline,time.monotonic())+3.)
                         if self.session_deadline is not None else 35.)
        params={'execution.enabled':'true','catalog':str(self.args.package_share/'config/environments.yaml'),
            'environment':s['raw']['environment'],'room':s['raw']['room'],'task_id':s['task'],
            'point_id':s['point']['id'],'point_index':str(s['point_index']),
            'map_sha256':s['prepared']['map_bundle_sha256'],'navigation.action_name':self.args.action_name,
            'lifetime_sec':str(worker_lifetime)}
        if self.session_deadline is not None:params['navigation.use_session_budget']='true'
        if s.get('turn_only'):
            params['navigation.search_behavior_tree']=str(self.args.turn_behavior_tree)
        command=[str(self.args.worker),'--ros-args']
        for key,value in params.items():command+=['-p',key+':='+value]
        for suffix in ('map_check/request','map_check/reply','lease/request','lease/reply','owned_goals',
                       'owned_goal_ack','request','execution_status','binding','retire'):
            command+=['-r','/handyman/search/'+suffix+':='+prefix+'/'+suffix]
        handle=(s['directory']/'worker.console.log').open('x');s['handles'].append(handle)
        s['worker']=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT)
        def can_retire():
            return self.observer_done(s) and all(k in s['manager'].results for k in registry.entries)
        s['supervisor']=SearchProcessSupervisor(self.node,SearchProcessWatchdog(s['task'],s['worker'],worker_lifetime),
            request_topic=prefix+'/request',status_topic=prefix+'/execution_status',goal_canceller=s['manager'],
            lease_topics=(prefix+'/lease/request',prefix+'/lease/reply'),
            retire_topic=prefix+'/retire',retirement_check=can_retire)
        self.log('worker_started',task_id=s['task'],pid=s['worker'].pid)

    def on_worker_status(self,msg):
        s=self.session
        if s is None:return
        try:row=yaml.safe_load(msg.detail)
        except yaml.YAMLError:return
        if not isinstance(row,dict) or row.get('task_id')!=s['task']:return
        if msg.message=='search_worker_fault':
            self.fail('search_runtime_failed')
        elif msg.message=='search_cancel_status' and row.get('state')=='cancel_failed':
            self.fail('search_runtime_failed')
        # Do NOT forward worker cancel_drained: observer/process retirement must finish first.

    def tick(self):
        s=self.session
        if self.recovery:
            remaining=len(self.journal.rows(unresolved=True))
            if remaining!=self.recovery_remaining:
                self.recovery_remaining=remaining
                self.log('recovery_progress',unresolved=remaining,dispatch_locked=True)
        if s is None:return
        try:
            if s['manager'] is not None:
                if s['manager'].failed:self.fail('search_runtime_failed')
                for key in s['manager'].registry.entries:
                    if key not in s.setdefault('logged_goals',set()):
                        s['logged_goals'].add(key)
                        self.log('goal_registered',task_id=s['task'],goal_id=key)
            if self.job is not None and self.job.done():
                expected,digest=self.job.result();self.job=None
                if self.gate.install(s['task'],expected,digest) and self.latest:
                    self.gate.receive_map(*self.latest,self.publishers())
            self.consumer.tick(time.monotonic());self.gate.check(self.publishers())
            if self.consumer.active is None and s['cancel'] is None:self.fail('search_runtime_failed')
            if s['fault'] is not None:
                if s.get('emergency_cancel'):
                    self.publish(s['cancel_pub'],'search_cancelled',s['emergency_cancel'])
                self.publish(self.status,'search_worker_fault',dict(schema='handyman-search-worker-fault-v1',
                    task_id=s['task'],reason=s['fault']))
                if s['cancel'] is not None:
                    self.publish(s['cancel_pub'],'search_cancelled',s['cancel'])
                    self.publish(self.status,'search_cancel_status',dict(schema='handyman-search-cancel-status-v1',
                        task_id=s['task'],cancel_id=s['cancel']['cancel_id'],state='cancel_failed'))
            if s['observer'] is not None:
                s['records'].poll(s['directory']/'observer.jsonl')
                head=s.get('head')
                if head is not None:
                    if s['cancel'] is not None or s['fault'] is not None or s['records'].terminal is not None:
                        head.cancel()
                    else:
                        head.tick(s['records'].arrived)
                        if head.failed:
                            self.log('head_stage_failed',task_id=s['task'],**head.snapshot())
                            self.fail('search_head_failed')
                if s['observer'].poll() is not None:
                    if not self.observer_done(s):self.fail('search_observer_failed')
                    elif s['cancel'] is None and s['fault'] is None:
                        s['candidate']=s['records'].terminal
                        proposal=None
                        if s.get('sequence') is not None:
                            proposal=copy.deepcopy(s['sequence']).complete_observation(dict(s['candidate'],cleanup_verified=True))
                        if proposal and proposal['state']=='awaiting_point' and proposal['point_id']!=s['point']['id']:
                            # Proposal is not committed until real worker retirement below.
                            s['local_transition']=True;s['cancel_time']=time.monotonic()
                            s['cancel']=dict(schema='handyman-search-request-v1',task_id=s['task'],
                                cancel_id=uuid.uuid4().hex,reason='point_observation_complete')
                            self.publish(s['cancel_pub'],'search_cancelled',s['cancel'])
                            self.log('point_transition_started',task_id=s['task'],point_id=s['point']['id'])
                        else:
                            self.publish(self.status,'search_stop_requested',dict(schema='handyman-search-stop-v1',
                                task_id=s['task'],reason='observation_terminal'))
            if s['cancel'] is not None:
                if s['worker'] is None:
                    done=s['observer'] is None or self.observer_done(s)
                else:done=s['supervisor'].watchdog.retired and self.observer_done(s)
                if s.get('head') is not None:done=done and s['head'].drained()
                if done and s['fault'] is None:
                    if s.get('local_transition') and not s.get('public_cancel'):self.next_point(s)
                    else:
                        if s.get('public_cancel'):s['cancel']=s['public_cancel']
                        self.finish(s)
                    return
                if time.monotonic()-s['cancel_time']>2.5:self.fail('search_runtime_failed')
                return
            if s['fault'] is not None:return
            if s['observer'] is None and self.consumer.active.get('live_map_verified'):self.start_observer(s)
            elif s['observer'] is not None and s['worker'] is None and s['records'].ready and s['observer'].poll() is None:
                if self.node.count_subscribers(s['prefix']+'/binding')>0:self.start_worker(s)
            if self.session_deadline is None and time.monotonic()-s['started']>30.:self.fail('search_runtime_failed')
        except Exception as exc:
            self.log('runtime_exception',reason=str(exc));self.fail('search_runtime_failed')

    def next_point(self,s):
        if self.active_binding:self.active_binding.check()
        # Consumer/map checks remain live throughout the private drain. Never
        # renew the original 30-second request lease just to permit more points.
        active=self.consumer.active
        if self.stopping or active is None or active['task_id']!=s['task'] or not active.get('live_map_verified'):
            self.fail('search_runtime_failed');return
        if self.journal.rows(unresolved=True):self.fail('search_runtime_failed');return
        decision=s['sequence'].complete_observation(dict(s['candidate'],cleanup_verified=True))
        if decision['state']!='awaiting_point':raise ValueError('continuation_no_longer_eligible')
        index=s['point_index']+1;point=s['prepared']['points'][index]
        if point['id']!=decision['point_id']:raise ValueError('continuation_point_mismatch')
        self.log('point_retired',task_id=s['task'],point_id=s['point']['id'],
                 observer_returncode=s['observer'].poll(),worker_returncode=s['worker'].poll())
        self.dispose(s)
        directory=self.args.output/s['task']/('point_'+str(index));directory.mkdir()
        prefix=self.namespace+'/task_'+s['task']+'/point_'+str(index)+'_'+uuid.uuid4().hex
        from rclpy.qos import QoSProfile,DurabilityPolicy
        fresh=dict(task=s['task'],raw=s['raw'],prepared=s['prepared'],point=point,point_index=index,
            sequence=s['sequence'],directory=directory,prefix=prefix,observer=None,worker=None,
            supervisor=None,manager=None,responder=None,private_sub=None,
            cancel_pub=self.node.create_publisher(self.Msg,prefix+'/request',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            cancel=None,cancel_time=None,candidate=None,fault=None,started=time.monotonic(),handles=[],
            records=ObserverRecords(s['task'],point['id'],s['prepared']['map_bundle_sha256'],s['raw']['target']))
        self.session=fresh
        from search_turn_policy import same_search_position
        fresh['turn_only']=bool(getattr(self.args,'turn_behavior_tree',None) and
                                same_search_position(s['point'],point))
        self.log('point_motion_mode',point_id=point['id'],
                 mode='turn_only' if fresh['turn_only'] else 'navigate')
        self.log('next_point_prepared',task_id=s['task'],point_id=point['id'],point_index=index)

    def finish(self,s):
        if self.active_binding:
            self.active_binding.check()
            if s['candidate'] is not None and not self.active_binding.result_matches(dict(s['candidate'],cleanup_verified=True)):
                self.fail('active_result_binding_mismatch');return
        self.publish(self.status,'search_cancel_status',dict(schema='handyman-search-cancel-status-v1',
            task_id=s['task'],cancel_id=s['cancel']['cancel_id'],state='cancel_drained'))
        if s['candidate'] is not None:
            row=dict(s['candidate'],schema='handyman-search-observation-v1',cleanup_verified=True)
            self.publish(self.results,'search_observation',row)
            (s['directory']/'result.json').write_text(json.dumps(row,indent=2))
        self.log('session_retired',task_id=s['task'],observer_returncode=s['observer'].poll() if s['observer'] else None,
            worker_returncode=s['worker'].poll() if s['worker'] else None,result_state=s['candidate']['state'] if s['candidate'] else None)
        self.dispose(s);self.session=None

    def dispose(self,s):
        if s.get('head'):s['head'].close()
        if s['supervisor']:s['supervisor'].close()
        if s['manager']:s['manager'].close()
        if s['responder']:
            self.node.destroy_subscription(s['responder'].subscription);self.node.destroy_publisher(s['responder'].publisher)
        if s['private_sub']:self.node.destroy_subscription(s['private_sub'])
        self.node.destroy_publisher(s['cancel_pub'])
        for handle in s['handles']:handle.close()

    def shutdown(self):
        self.stopping=True
        s=self.session
        if s is not None:
            import rclpy
            try:
                self.fail('search_runtime_failed')
                self.stop_lease(s)
                end=time.monotonic()+3.
                while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.05)
            except Exception as exc:
                # ROS shutdown must never bypass owned-child cleanup.
                self.log('shutdown_ros_unavailable',reason=str(exc),cleanup_verified=False)
            for process in (s['observer'],s['worker']):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:process.wait(timeout=2)
                    except subprocess.TimeoutExpired:process.kill();process.wait(timeout=2)
            self.log('forced_shutdown',task_id=s['task'],cleanup_verified=False)
            self.dispose(s)
        for recovery in self.recovery:recovery.close()
        self.pool.shutdown(wait=False,cancel_futures=True);self.journal.close();self.events.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true');p.add_argument('--allow-live',action='store_true')
    p.add_argument('--package-share',type=Path,required=True);p.add_argument('--worker',type=Path,required=True)
    p.add_argument('--decoder',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--journal',type=Path,help='Stable persistent SQLite path; reuse after restart, never a new path to bypass recovery')
    p.add_argument('--action-name',default='/navigate_to_pose');p.add_argument('--map-topic',default='/map')
    p.add_argument('--vision-topic',default='/handyman/vision/diagnostics')
    p.add_argument('--request-topic',default='/handyman/search/request')
    p.add_argument('--status-topic',default='/handyman/search/execution_status')
    p.add_argument('--result-topic',default='/handyman/search/result')
    p.add_argument('--private-prefix',default='/handyman/search/runtime')
    p.add_argument('--point-index',type=int,default=0)
    p.add_argument('--active-bundle',type=Path,help='Bind exact task and eight-point snapshot; does not enable live multi-point mode')
    p.add_argument('--enable-active-ring',action='store_true',help='Explicit bound live ring; requires private topics and current path preflight')
    p.add_argument('--check-current-path',action='store_true',help='Also exercise bound read-only planner preflight in isolated tests')
    p.add_argument('--multi-point',action='store_true',help='Experimental domain-73-only automatic continuation')
    p.add_argument('--head-tilt',type=float,help='Opt-in automatic head stage; live requires allow-live and domain 71')
    p.add_argument('--turn-behavior-tree',type=Path,
                   help='Opt-in search-only turn BT for consecutive colocated ring views')
    p.add_argument('--observation-seconds',type=float,default=20.)
    p.add_argument('--view-seconds',type=float,help='Separate per-view observation deadline; does not enable multi-point execution')
    p.add_argument('--seconds',type=float,default=600.)
    p.add_argument('--session-budget-seconds',type=float,
                   help='Actual remaining session time; replaces short 25/30/35 s navigation limits')
    p.add_argument('--session-event-topic',default='/handyman/message/to_robot')
    a=p.parse_args()
    if a.turn_behavior_tree:
        if not a.active_bundle or not a.multi_point or not a.turn_behavior_tree.is_file():
            p.error('turn behavior requires an existing BT file and bound multi-point ring')
        a.turn_behavior_tree=a.turn_behavior_tree.resolve()
    if a.check_current_path and not a.active_bundle:p.error('current path check requires active-bundle')
    if a.active_bundle and (not a.multi_point or a.point_index!=0 or a.session_budget_seconds is None):
        p.error('active-bundle requires multi-point, index zero and session budget')
    if not a.execute:print('Search runtime disabled; pass --execute explicitly.');return
    if a.journal is None:p.error('--journal is required for execution/recovery')
    if a.allow_live and a.session_budget_seconds is None:
        p.error('live execution requires explicit remaining --session-budget-seconds; short test deadlines are disabled for live use')
    if a.session_budget_seconds is not None:
        try:a.session_budget_seconds=session_budget(a.session_budget_seconds)
        except ValueError as exc:p.error(str(exc))
        if a.view_seconds is None:p.error('session budget requires explicit view-seconds')
        a.seconds=a.session_budget_seconds
    if a.head_tilt is not None:
        from head_view_trial import target
        from search_head_stage import validate_environment
        target(0.,a.head_tilt)
        validate_environment(os.environ.get('ROS_DOMAIN_ID'),os.environ.get('ROS_LOCALHOST_ONLY'),a.allow_live)
    if a.enable_active_ring:
        from active_live_entry import validate_live_ring
        try:validate_live_ring(a,os.environ)
        except ValueError as exc:p.error(str(exc))
    if a.multi_point and not a.enable_active_ring and (a.allow_live or os.environ.get('ROS_DOMAIN_ID')!='73' or
        os.environ.get('ROS_LOCALHOST_ONLY')!='1' or a.action_name!='/handyman_test/navigate_to_pose'):
        p.error('multi-point is isolated-test-only')
    if a.multi_point and a.view_seconds is None:p.error('multi-point requires an explicit view-seconds')
    if not a.allow_live and (os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or
        a.action_name!='/handyman_test/navigate_to_pose'):p.error('non-test execution requires --allow-live')
    if not 3<=a.observation_seconds<=25 or not 1<=a.seconds<=3600 or a.point_index<0:p.error('invalid timing/index')
    if a.view_seconds is not None and not 0<a.view_seconds<a.observation_seconds:p.error('invalid view deadline')
    a.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('handyman_search_runtime');runtime=SearchRuntime(node,a)
    def request_shutdown(signum,frame):runtime.stopping=True
    signal.signal(signal.SIGINT,request_shutdown);signal.signal(signal.SIGTERM,request_shutdown)
    try:
        end=runtime.session_deadline or time.monotonic()+a.seconds
        while rclpy.ok() and not runtime.stopping and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:runtime.shutdown();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()

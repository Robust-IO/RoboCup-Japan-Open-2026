"""Manual single-task owner. Default preflight; --run can cause robot motion.

Uses only private search topics, never competition protocol topics. An existing
search runtime must own navigation/cancellation. This tool is not an emergency stop.
"""
import argparse
import json
import signal
from pathlib import Path
import time
import uuid
import yaml
from build_search_plan import build
from search_timing import session_budget,SessionEvents


def selected_point(body,index):
    if type(index) is not int or not 0<=index<len(body['points']):
        raise ValueError('point_index_out_of_range')
    return body['points'][index]


def request_body(share,task,point_index=2):
    env=yaml.safe_load((share/'config/environments/layout_a.yaml').read_text())
    room=next(r for r in build(env)['rooms'] if r['room']=='living_room')
    if not room['valid']:raise ValueError('invalid living room plan')
    body=dict(schema='handyman-search-request-v1',task_id=task,environment='LayoutA',layout='LayoutA',
                room='living_room',target='canned_juice',map_uri=env['map'],frame_id='map',
                points=[dict(id=p['id'],pose=p['pose']) for p in room['points']],
                actionable=False,requires_map_verification=True)
    selected_point(body,point_index)
    return body


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package-share',type=Path,required=True)
    p.add_argument('--active-bundle',type=Path,help='Use the bundle task and ring; preflight only until live ring is enabled')
    p.add_argument('--enable-active-ring',action='store_true')
    p.add_argument('--run',action='store_true');p.add_argument('--confirm-motion',action='store_true')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--point-index',type=int,default=2,
                   help='Existing living-room YAML index; runtime --point-index must match')
    p.add_argument('--session-budget-seconds',type=float,help='Actual remaining session time, replaces 32 s owner wait')
    p.add_argument('--session-event-topic',default='/handyman/message/to_robot')
    args=p.parse_args();task=uuid.uuid4().hex
    binding=None
    try:
        if args.active_bundle:
            from active_run_binding import ActiveRunBinding
            binding=ActiveRunBinding(args.active_bundle,args.package_share)
            if args.point_index!=0:p.error('active-bundle requires explicit --point-index 0')
            body=dict(binding.bundle['request']);task=body['task_id']
        else:body=request_body(args.package_share,task,args.point_index)
    except ValueError as exc:p.error(str(exc))
    point=selected_point(body,args.point_index)
    if args.session_budget_seconds is not None:
        try:args.session_budget_seconds=session_budget(args.session_budget_seconds)
        except ValueError as exc:p.error(str(exc))
    if not args.run:
        print(json.dumps(dict(preflight=True,target=body['target'],selected_point=point,
                             active_bundle=bool(binding),point_count=len(body['points']),
                             required_runtime_point_index=args.point_index,motion_started=False,
                             session_budget_seconds=args.session_budget_seconds)));return
    if not args.confirm_motion:p.error('--run requires --confirm-motion')
    if bool(binding)!=args.enable_active_ring:p.error('live ring requires both active-bundle and enable-active-ring')
    if args.session_budget_seconds is None:p.error('live run requires explicit remaining --session-budget-seconds')
    import os
    if os.environ.get('ROS_DOMAIN_ID')!='71':p.error('This manual live owner requires domain 71')
    if binding and os.environ.get('ROS_LOCALHOST_ONLY')!='1':p.error('live ring requires localhost-only DDS')
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from handyman_msgs.msg import HandymanMsg
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    prefix='/handyman/manual_single_search'
    with args.output.open('x') as stream:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('handyman_manual_single_search')
        pub=node.create_publisher(HandymanMsg,prefix+'/request',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cancel=None;result=None;failed=False;sent=False;drained=False
        interrupted=False
        def interrupt(signum,frame):
            nonlocal interrupted,failed
            interrupted=True;failed=True
        signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
        def log(event,**values):
            stream.write(json.dumps(dict(event=event,time=time.monotonic(),**values),allow_nan=False)+'\n');stream.flush()
        def send(event,row):
            m=HandymanMsg();m.message=event;m.detail=yaml.safe_dump(row);pub.publish(m)
        def stop(reason):
            nonlocal cancel
            if cancel is None:
                cancel=dict(schema='handyman-search-request-v1',task_id=task,cancel_id=uuid.uuid4().hex,reason=reason)
                log('cancel_requested',**cancel)
            send('search_cancelled',cancel)
        def receive(m):
            nonlocal result,failed,drained
            try:row=yaml.safe_load(m.detail)
            except yaml.YAMLError:return
            if not isinstance(row,dict) or row.get('task_id')!=task:return
            log('received',message=m.message,data=row)
            if m.message=='search_stop_requested':stop('search_stop:observation_terminal')
            elif m.message=='search_worker_fault':failed=True;stop('manual_owner_fault')
            elif m.message=='search_cancel_status' and cancel and row.get('cancel_id')==cancel['cancel_id']:
                if row.get('state')=='cancel_drained':drained=True
                elif row.get('state')=='cancel_failed':failed=True
            elif m.message=='search_observation':
                context=row.get('observer_context',{})
                if binding:
                    if binding.result_matches(row):result=row
                elif (row.get('cleanup_verified') is True and row.get('target')==body['target'] and
                    context.get('point_id')==point['id'] and context.get('task_id')==task and
                    row.get('actionable') is False and row.get('does_not_exist_authorized') is False):result=row
        subscriptions=[node.create_subscription(HandymanMsg,prefix+'/status',receive,10),
                       node.create_subscription(HandymanMsg,prefix+'/result',receive,10)]
        if args.session_budget_seconds is not None:
            session_event_state=SessionEvents()
            def session_event(msg):
                nonlocal interrupted,failed
                if session_event_state.stops(msg.message,msg.detail):
                    interrupted=True;failed=True
                    if sent:stop('session_ended_or_changed')
            subscriptions.append(node.create_subscription(HandymanMsg,args.session_event_topic,session_event,10))
        try:
            end=time.monotonic()+5
            while pub.get_subscription_count()!=1 and time.monotonic()<end and not interrupted:rclpy.spin_once(node,timeout_sec=.05)
            if interrupted:raise KeyboardInterrupt()
            if pub.get_subscription_count()!=1:raise RuntimeError('Need exactly one private runtime request subscriber')
            if binding:binding.check()
            body['stamp_ns']=node.get_clock().now().nanoseconds
            send('search_requested',body);sent=True;log('request_sent',task_id=task,point_id=point['id'])
            end=time.monotonic()+args.session_budget_seconds
            while rclpy.ok() and time.monotonic()<end and result is None and not failed:
                rclpy.spin_once(node,timeout_sec=.05)
        except (KeyboardInterrupt,ExternalShutdownException):failed=True
        finally:
            if sent and not drained:
                stop('manual_owner_finished')
                end=time.monotonic()+5
                while rclpy.ok() and not drained and time.monotonic()<end:
                    rclpy.spin_once(node,timeout_sec=.05)
            log('owner_finished',cleanup_verified=drained,result=result,failed=failed)
            print(json.dumps(dict(task_id=task,cleanup_verified=drained,result=result,failed=failed)))
            node.destroy_node();rclpy.try_shutdown()
        if failed or result is None or not drained:raise SystemExit(1)


if __name__=='__main__':main()

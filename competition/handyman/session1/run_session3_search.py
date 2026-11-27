"""Session3 bear search, corrected sofa dog grasp, and held delivery to Moderator."""
import argparse,json,signal,uuid,time,sys
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path);p.add_argument('--execute',action='store_true');resume=p.add_mutually_exclusive_group();resume.add_argument('--resume-room-from',type=Path);resume.add_argument('--resume-held-from',type=Path);p.add_argument('--prepare-initial-empty',action='store_true');a=p.parse_args()
 if a.prepare_initial_empty and (not a.execute or a.resume_room_from or a.resume_held_from):p.error('initial preparation requires fresh execute without resume')
 cfg=json.loads(a.config.read_text())
 from live_config import validate
 validate(cfg)
 # Plan/config content is checked without initializing ROS or sending commands.
 import hashlib
 if hashlib.sha256(Path(cfg['environment']).read_bytes()).hexdigest()!=cfg['environment_sha256']:raise ValueError('search_environment_changed_rebuild_plan')
 plan=cfg['search_plan']
 if hashlib.sha256(Path(plan['map_yaml']).read_bytes()+b'\0'+Path(plan['map_image']).read_bytes()).hexdigest()!=plan['map_bundle_sha256']:raise ValueError('search_map_changed_rebuild_plan')
 from handover_config import validate_handover
 validate_handover(cfg,check_sources=True)
 if not a.execute:print(json.dumps(cfg['search_plan']));return
 if a.output is None:p.error('--output required')
 from runtime_resources import RuntimeResources
 from motion_stop import MotionGate,StopController
 from ros_protocol import bind_application
 from session3_search_live import Session3Search
 owner=RuntimeResources(a.config,Path('/__HANDYMAN_PACKAGE__/runtime/autograsp-v1/replay-inputs/hand.json'),a.output)
 b=owner.open(prepare_initial_empty=a.prepare_initial_empty);app=protocol=None;protocol_executor=None;interrupted=False
 def stop(*args):
  nonlocal interrupted
  interrupted=True
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
 try:
  runtime=b['runtime'];gate=MotionGate();gate.install(runtime,b['motion_backend']);controller=StopController(runtime,b['motion_backend'],gate,a.output)
  search=Session3Search(b)
  from stage_bindings import StageBindings
  from candidate_selection import load_selector
  from hold_observer import HoldObserver
  from withdrawal_observer import WithdrawalObserver
  from table_withdrawal import TableWithdrawal
  from transfer_actions import tuck
  stages=StageBindings(runtime,b['motion_backend'],b['grasp_backend'],cfg,b['environment'],a.output,
      candidate_selector=load_selector(cfg,b['semantic']))
  from shared_turn_guard import install
  install(stages)
  from handover_live import HandoverLive
  handover=HandoverLive(stages)
  def carry_ready(effect,cancel):
   stages.check(cancel)
   stages.context['require_visual_carry_checks']=False
   result=tuck(runtime,observe_hold=HoldObserver(b['grasp_backend'],stages.context),
       withdraw_from_source=TableWithdrawal(runtime,b['motion_backend'],
           WithdrawalObserver(b['grasp_backend'],b['grasp_backend'].table,lambda:stages.context),label='session3-source-withdraw'))
   stages.check(cancel);return result
  handlers={'navigate_room':search.navigate_room,'search':search.search,'navigate_source':stages.navigate_source,
      'observe_target':stages.observe_target,'grasp':stages.grasp,'tuck':carry_ready,
      'navigate_moderator':handover.navigate,'handover':handover.present}
  app,protocol=bind_application(b['node'],'session3-'+uuid.uuid4().hex,handlers,controller.request_stop,profile=cfg['session_profile']);controller.bind(app)
  from rclpy.executors import SingleThreadedExecutor
  protocol_executor=SingleThreadedExecutor();protocol_executor.add_node(b['node'])
  runtime.sample(profile='state');state=runtime.state()
  if b['motion_backend'].room_at(state['poses']['map']) is None:raise ValueError('localized_start_required')
  if not a.resume_held_from and state['joints'][5]<.6:raise ValueError('empty_hand_start_required')
  if a.resume_held_from:
   from resume_session3_held import restore
   restored=restore(app.session,stages,a.resume_held_from,state)
   (a.output/'resume.json').write_text(json.dumps(restored,indent=2))
  elif a.resume_room_from:
   from resume_session3_room import restore
   restored=restore(app.session,a.resume_room_from)
   (a.output/'resume.json').write_text(json.dumps(restored,indent=2))
  else:app.session.preflight(ready=True)
  import rclpy
  while True:
   if interrupted:app.session.stop('operator_cancelled')
   protocol_executor.spin_once(timeout_sec=.05)
   (a.output/'live-state.json').write_text(json.dumps(app.session.snapshot(),indent=2))
   if app.session.state in app.session.TERMINAL and app.executor.retired:break
  report=app.session.snapshot()
  (a.output/'session-result.json').write_text(json.dumps(report,indent=2))
  print(json.dumps(report))
  return 0 if report['state']=='succeeded' else 2
 finally:
  if app is not None and not app.executor.retired:
   app.session.stop('runner_exit')
   import rclpy
   while not app.executor.retired:protocol_executor.spin_once(timeout_sec=.05)
  if protocol_executor is not None:
   protocol_executor.remove_node(b['node']);protocol_executor.shutdown()
  owner.close(app,protocol)
if __name__=='__main__':raise SystemExit(main())

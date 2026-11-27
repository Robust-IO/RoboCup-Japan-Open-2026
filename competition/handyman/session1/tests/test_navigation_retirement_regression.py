import unittest,tempfile,json,math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch,Mock
from threading import Event
from shared_turn_guard import SharedTurnGuard
from motion_stop import MotionGate,StopController
from round_retirement import RoundRetirement
class RecoveryTests(unittest.TestCase):
 def test_archive_remains_available_to_stop_confirmation(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);f=root/'nav.json';f.write_text(json.dumps({'nav_reset':True,'carry_footprint_restored':True}))
   gate=MotionGate();gate.child_command_uncertain=True;gate.navigation_report_path=f
   guard=SharedTurnGuard.__new__(SharedTurnGuard);guard.motion=SimpleNamespace(session_motion_gate=gate)
   archived=guard.archive_report(f,'-blocked-turn-1')
   self.assertEqual(gate.navigation_report_path,archived);self.assertFalse(f.exists())
   stamps=iter([1,200000001,400000001])
   rt=SimpleNamespace(pump=lambda dt:None,state=lambda:dict(stamp_ns=next(stamps),joints=[0]*6,poses={'odom':[0,0,0]}))
   c=StopController(rt,SimpleNamespace(active_child=None),gate,root);c.navigation_report_path=f
   try:self.assertTrue(c._verify({'operation_id':'x/1'},True)['motion_stopped'])
   finally:c.close()
 def test_escape_failure_replans_and_restores_stage(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);out=root/'nav.json';events=[];cancel=Event();guard=SharedTurnGuard.__new__(SharedTurnGuard)
   guard.cancel=cancel;guard.busy=False;guard.envelope=lambda:'envelope';guard.geometry=lambda:None
   guard.motion=SimpleNamespace(stage='navigate_destination')
   rt=SimpleNamespace(sample=lambda **kw:None,state=lambda:{'poses':{'map':[0,0,0]}})
   guard.stages=SimpleNamespace(motion=guard.motion,runtime=rt,check=lambda c:None,grasp_backend=SimpleNamespace(record=events.append))
   def escape(*a,**kw):
    self.assertAlmostEqual(kw['target_yaw'],math.pi/2)
    guard.motion.stage='withdraw'
    raise ValueError('no_translation_only_carry_escape_on_static_map')
   guard.turner=SimpleNamespace(escape=escape);calls=[]
   def command(args,*a,**kw):
    calls.append(args)
    if len(calls)==1:
     out.write_text(json.dumps(dict(error='carry_replan_required:turn_sweep_blocked',nav_reset=True,carry_footprint_restored=True)))
     raise RuntimeError('failed')
    self.assertFalse(out.exists());self.assertEqual(guard.motion.stage,'navigate_destination')
    return {'status':'arrived'}
   guard.original_command=command
   with patch('carry_drop_recovery.enabled',return_value=False):
    self.assertEqual(guard.command(['carry_navigation_runtime.py','--goal',0,1,0,'--output',out])['status'],'arrived')
   self.assertTrue(any(e['event']=='navigation_escape_retry' for e in events))
 def test_heading_skips_reached_waypoint(self):
  heading=SharedTurnGuard.recovery_heading(['--waypoint',1,0,0,'--waypoint',0,2,0,'--goal',3,3,0],{'reached':[{'target':[1,0,0]}]},[0,0,0])
  self.assertAlmostEqual(heading,math.pi/2)
 def test_retirement_waits_for_child_and_ignores_reused_pid(self):
  rows={10:(1,'a','S'),11:(10,'b','S'),99:(1,'z','S')};clock=[0.];signals=[]
  with patch('round_retirement.identities',side_effect=lambda:rows.copy()),patch('round_retirement.os.kill',side_effect=lambda p,s:signals.append((p,s))):
   ret=RoundRetirement(10,lambda **kw:None,clock=lambda:clock[0])
   self.assertEqual({p for p,s in signals},{10,11})
   rows.pop(10);rows[11]=(1,'b','S');clock[0]=16
   self.assertFalse(ret.poll())
   rows[11]=(1,'new-birth','S');clock[0]=21;signals.clear()
   self.assertTrue(ret.poll());self.assertEqual(signals,[])

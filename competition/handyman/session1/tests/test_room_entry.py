import tempfile,unittest,json
from pathlib import Path
from types import SimpleNamespace
from room_entry import entered,finish_on_room_entry
from session1 import Session1
class RoomEntryTests(unittest.TestCase):
 def test_membership_does_not_require_furniture_position_or_heading(self):
  region={'polygon':[[0,0],[4,0],[4,4],[0,4]]}
  self.assertFalse(entered([.01,.2,2.7],region));self.assertTrue(entered([.5,.5,2.7],region));self.assertFalse(entered([.49,2,0],region));self.assertFalse(entered([-.01,.2,0],region))
 def test_only_scoped_room_navigation_uses_boundary_completion(self):
  calls=[];original=lambda args,*a,**kw:calls.append(args)
  motion=SimpleNamespace(command=original)
  with tempfile.TemporaryDirectory() as t:
   with finish_on_room_entry(motion,t,'lobby',[[0,0],[4,0],[4,4]]):
    motion.command([Path('/old/navigate_carry_trial.py'),'--run'])
    self.assertEqual(Path(calls[0][0]).name,'room_entry_navigation.py');self.assertIn('--room-region',calls[0])
   self.assertIs(motion.command,original)
 def test_scope_restores_original_command_on_failure(self):
  original=lambda *a:None;motion=SimpleNamespace(command=original)
  with tempfile.TemporaryDirectory() as t:
   with self.assertRaises(RuntimeError):
    with finish_on_room_entry(motion,t,'lobby',[[0,0],[4,0],[4,4]]):raise RuntimeError('failed')
   self.assertIs(motion.command,original)
 def test_room_protocol_precedes_source_navigation(self):
  p={'layout':'LayoutC','target':'tumbler','source':{'room':'lobby','furniture_id':'white_side_table#lobby'},'destination':{'room':'lobby','furniture_id':'trash_box_for_burnable#lobby','mode':'container_drop'}}
  s=Session1('test',profile=p);s._request('navigate_room');s.drain();op=s.pending
  s.result(op,dict(status='ok',room='lobby',arrived=True),session_id=s.session_id,run_id=s.run_id)
  events=s.drain();self.assertEqual(events[0]['message'],'Room_reached');self.assertEqual(events[1]['stage'],'navigate_source')
 def test_search_room_entry_does_not_go_to_main_when_already_inside(self):
  from session3_search_live import Session3Search
  from threading import Event
  s=Session3Search.__new__(Session3Search);s.room="lobby"
  s.rt=SimpleNamespace(sample=lambda **kw:None,state=lambda:{'poses':{'map':[1,1,0]}})
  s.b={'environment':{'rooms':{'lobby':{'region':[[0,0],[4,0],[4,4],[0,4]]}}}}
  s.motion=SimpleNamespace(room_at=lambda p:'lobby');s.navigate=lambda *a:self.fail('entry must not visit main point')
  self.assertTrue(s.navigate_room({},Event())['arrived'])

 def test_furniture_proximity_ignores_final_pose_and_heading(self):
  r={'mode':'near_furniture','polygon':[[-5,-5],[5,-5],[5,5],[-5,5]],'bounds':{'center':{'x':0,'y':0},'size':{'x':2,'y':1}},'distance_m':1.0}
  self.assertTrue(entered([1.8,0,2.8],r))
  self.assertFalse(entered([2.01,0,0],r))

 def test_navigation_endpoint_can_trigger_observation_before_distance(self):
  from room_entry import completion_basis
  r={'mode':'near_furniture','polygon':[[-5,-5],[5,-5],[5,5],[-5,5]],'bounds':{'center':{'x':0,'y':0},'size':{'x':2,'y':1}},'distance_m':1.0}
  self.assertIsNone(completion_basis([2.35,0,0],r))
  self.assertEqual(completion_basis([2.35,0,0],r,final_goal_reached=True),'furniture_navigation_goal_reached_first')
  self.assertEqual(completion_basis([1.9,0,2],r),'near_furniture_current_view')
  self.assertIsNone(completion_basis([6,0,0],r,final_goal_reached=True))
  room={'polygon':r['polygon']}
  self.assertIsNone(completion_basis([4.8,0,0],room,final_goal_reached=True))

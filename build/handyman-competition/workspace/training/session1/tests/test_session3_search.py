import json,unittest
from pathlib import Path
from session3 import Session3
from session3_search_policy import aspect_ratio,choose_points
P=json.loads((Path(__file__).resolve().parents[1]/'profiles/session3.json').read_text())
class SessionTests(unittest.TestCase):
 def start(self):
  s=Session3('s',profile=P);s.preflight(ready=True)
  for m,d in [('Environment','LayoutA'),('Are_you_ready?',''),('Instruction',P['expected_instruction'])]:s.moderator(m,d,session_id='s')
  s.drain();s.result(s.pending,dict(status='ok',arrived=True,room='lobby'),session_id='s',run_id=s.run_id);s.drain();return s
 def result(self,s,**kw):
  r=dict(status='ok',target=s.task.target,room='lobby',evidence_ref='capture',confirmed=False,search_exhausted=True,valid_views=16,required_views=16,unresolved_candidates=0);r.update(kw)
  s.result(s.pending,r,session_id='s',run_id=s.run_id)
 def test_absence_then_correction_then_found(self):
  s=self.start();self.result(s);self.assertEqual(s.state,'waiting_correction');self.assertEqual(s.drain()[0]['message'],'Does_not_exist')
  s.moderator('Corrected_instruction',P['corrected_instruction'],session_id='s');self.assertEqual(s.task.target,'dog_doll')
  self.assertEqual(s.state,'navigate_source');self.assertEqual(s.task.source_table,'corner_sofa#lobby');s.drain()
  for result in [dict(arrived=True,room='lobby',source_table='corner_sofa#lobby'),
                 dict(confirmed=True,target='dog_doll',room='lobby',source_table='corner_sofa#lobby',evidence_ref='rgbd'),
                 dict(target='dog_doll',grasp_verified=True),dict(holding_verified=True,carry_ready=True)]:
   s.result(s.pending,dict(status='ok',**result),session_id='s',run_id=s.run_id)
  self.assertEqual(s.state,'navigate_moderator')
  events=s.drain();self.assertTrue(any(e.get('message')=='Object_grasped' for e in events));self.assertFalse(any(e.get('message')=='Task_finished' for e in events))
 def delivery(self):
  s=self.start();s.corrected=True
  from dataclasses import replace
  s.task=replace(s.task,target='dog_doll');s._request('handover');s.drain();return s
 def test_delivery_waits_for_official_success(self):
  s=self.delivery()
  self.result(s,destination='Moderator',holding_verified=True,delivery_ready=True,release_sent=False)
  self.assertEqual(s.state,'waiting_moderator')
  self.assertEqual([e['message'] for e in s.drain() if e['kind']=='protocol'],['Task_finished'])
  s.moderator('Task_succeeded',session_id='s');self.assertEqual(s.state,'succeeded')
 def test_released_or_unconfirmed_hand_never_finishes(self):
  for changes in [dict(release_sent=True),dict(holding_verified=False),dict(delivery_ready=False),dict(target='bear_doll')]:
   s=self.delivery();r=dict(destination='Moderator',holding_verified=True,delivery_ready=True,release_sent=False);r.update(changes)
   self.result(s,**r);self.assertEqual(s.state,'stopped');self.assertFalse(any(e.get('message')=='Task_finished' for e in s.drain()))
 def test_cancelled_delivery_ignores_late_result(self):
  s=self.delivery();op=s.pending;s.stop('session ended');s.drain()
  s.result(op,dict(status='ok',target='dog_doll',destination='Moderator',holding_verified=True,delivery_ready=True,release_sent=False,evidence_ref='proof'),session_id='s',run_id=s.run_id)
  self.assertEqual(s.state,'stopped');self.assertEqual(s.drain(),[])
 def test_incomplete_not_absence(self):
  s=self.start();self.result(s,valid_views=15);self.assertEqual(s.state,'stopped');self.assertFalse(any(e.get('message')=='Does_not_exist' for e in s.drain()))
 def test_tentative_not_absence(self):
  s=self.start();self.result(s,unresolved_candidates=1);self.assertEqual(s.state,'stopped')
 def test_early_correction_cancels(self):
  s=self.start();s.moderator('Corrected_instruction',P['corrected_instruction'],session_id='s');self.assertEqual(s.state,'stopped')
 def test_other_room_not_accepted(self):
  s=self.start();self.result(s,room='living_room');self.assertEqual(s.state,'stopped')
 def test_stale_result_ignored(self):
  s=self.start();s.result('old',{},session_id='s',run_id=s.run_id);self.assertEqual(s.state,'search')
class ShapeTests(unittest.TestCase):
 def test_rotated_square(self):
  r,_=aspect_ratio([[0,1],[1,0],[2,1],[1,2]]);self.assertAlmostEqual(r,1)
 def test_main_only_square_backup_elongated(self):
  row={'main':{'x':0,'y':0,'yaw':0},'backups':[{'x':1,'y':0,'yaw':0}]}
  self.assertEqual(len(choose_points(row,[[0,0],[2,0],[2,2],[0,2]])['points']),1)
  self.assertEqual(len(choose_points(row,[[0,0],[4,0],[4,2],[0,2]])['points']),2)

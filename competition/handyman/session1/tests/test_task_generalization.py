import json,unittest
from copy import deepcopy
from pathlib import Path
from task_generalization import compile_all,validate_generalized
from session1 import Session1
from session3 import Session3
ROOT=Path(__file__).resolve().parents[4]
class ConfigurableTasksTests(unittest.TestCase):
 def document(self):return json.loads((ROOT/'competition_tasks.json').read_text())
 def test_defaults_and_recovery_scope(self):
  configs=compile_all(ROOT,self.document())
  self.assertEqual(len(configs),6)
  for c in configs.values():
   n=c['session_profile']['session_number']
   self.assertEqual(c['policy']['carry_drop_recovery'],n in (2,5,6))
   validate_generalized(c)
 def test_object_source_and_destination_change_together(self):
  doc=self.document();spec=doc['sessions'][5]
  spec['target']='apple';spec['expected_instruction']='Go to the lobby, grasp the apple and put it on the square_low_table.'
  spec['destination']={'mode':'table_place','room':'living_room','furniture_id':'square_low_table#living_room'}
  c=compile_all(ROOT,doc)['session6-live-config.json'];session=Session1('test',profile=c['session_profile'])
  self.assertEqual(session.task.target,'apple');self.assertEqual(session.task.destination,'square_low_table#living_room')
  self.assertEqual(c['task']['object'],'apple');self.assertEqual(c['destination_id'],session.task.destination)
 def test_unknown_object_or_furniture_rejected(self):
  for role,value in [('target','untrained_object'),('source',{'room':'lobby','furniture_id':'missing#lobby'})]:
   doc=self.document();doc['sessions'][5][role]=value
   with self.assertRaises(ValueError):compile_all(ROOT,doc)
 def test_unmapped_container_rejected(self):
  doc=self.document();doc['sessions'][5]['destination']={'mode':'container_drop','room':'living_room','furniture_id':'square_low_table#living_room'}
  with self.assertRaisesRegex(ValueError,'calibrated_container_geometry_required'):compile_all(ROOT,doc)
 def test_corrected_session3_target_not_hardcoded(self):
  doc=self.document();spec=doc['sessions'][2];spec['target']='apple';spec['corrected_instruction']='Go to the lobby, grasp the apple and give it to me.'
  c=compile_all(ROOT,doc)['session3-search-config.json'];session=Session3('test',profile=c['session_profile']);session.state='waiting_correction'
  session.moderator('Corrected_instruction',spec['corrected_instruction'],session_id='test')
  self.assertEqual(session.task.target,'apple');self.assertEqual(session.task.source_table,spec['source']['furniture_id']);self.assertEqual(session.state,'navigate_source')
 def test_inconsistent_runtime_fields_rejected(self):
  c=compile_all(ROOT,self.document())['session6-live-config.json'];c['task']['object']='salt'
  with self.assertRaisesRegex(ValueError,'task_fields_not_synchronized'):validate_generalized(c)

 def test_source_room_and_furniture_change_together(self):
  doc=self.document();spec=doc['sessions'][5]
  spec['source']={'room':'living_room','furniture_id':'square_low_table#living_room'}
  spec['expected_instruction']='Go to the living room, grasp the filled_ketchup and put it on the wooden_shelf.'
  c=compile_all(ROOT,doc)['session6-live-config.json']
  self.assertEqual(c['task']['source_room'],'living_room')
  self.assertEqual(c['source_table_id'],'square_low_table#living_room')
  self.assertEqual(c['session_profile']['source'],spec['source'])
 def test_changed_instruction_starts_configured_task(self):
  doc=self.document();spec=doc['sessions'][5];spec['target']='apple';spec['expected_instruction']='Go to the lobby, grasp the apple and put it on the wooden_shelf.'
  c=compile_all(ROOT,doc)['session6-live-config.json'];session=Session1('test',profile=c['session_profile'])
  session.preflight(ready=True);session.moderator('Environment','LayoutD',session_id='test');session.moderator('Are_you_ready?',session_id='test');session.drain()
  session.moderator('Instruction',spec['expected_instruction'],session_id='test')
  self.assertEqual(session.state,'navigate_room');self.assertEqual(session.drain()[0]['task']['target'],'apple')

import json,unittest
from pathlib import Path
from dataclasses import asdict
import yaml
from instruction_task import parse_instruction,source_route
from dynamic_session import DynamicSession
from task_generalization import config_name
ROOT=Path(__file__).resolve().parents[4]
class DynamicTaskTests(unittest.TestCase):
 def setUp(self):
  names=yaml.safe_load((ROOT/'project/training/dataset.yaml').read_text())['names'];self.classes=list(names.values()) if isinstance(names,dict) else names
 def parse(self,n,text):
  c=json.loads((ROOT/'workspace/training/session1'/config_name(n)).read_text());sem=json.loads(Path(c['semantic_map']).read_text());env=yaml.safe_load(Path(c['environment']).read_text())
  return parse_instruction(text,self.classes,env['rooms'],sem['destinations'])
 def test_all_six_default_sentences_and_correction(self):
  doc=json.loads((ROOT/'competition_tasks.json').read_text())
  for s in doc['sessions']:
   t=self.parse(s['session_number'],s['expected_instruction'])
   self.assertEqual(t['target'],s.get('initial_target',s['target']))
   self.assertEqual(t['destination']['mode'],s['destination']['mode'])
   if s['session_number']==3:self.assertEqual(self.parse(3,s['corrected_instruction'])['target'],s['target'])
 def test_no_furniture_absent_record_requires_room_search(self):
  t=self.parse(3,'Go to the lobby, grasp the bear_doll and give it to me.')
  inv=json.loads((ROOT/'config/room-inventories/session3.json').read_text());r=source_route(t,inv)
  self.assertEqual(r['route'],'room_search');self.assertFalse(r['absence_authorized'])
 def test_present_but_unbound_uses_candidates(self):
  t=self.parse(1,'Go to the living room, grasp the dog_doll and put it on the square_low_table.')
  inv={'complete':True,'objects':[{'target':'dog_doll','room':'living_room','furniture_id':None}]}
  self.assertEqual(source_route(t,inv)['route'],'candidates_then_room')
 def test_unknown_inventory_never_assumes_absence(self):
  t=self.parse(1,'Go to the living room, grasp the dog_doll and put it on the square_low_table.')
  self.assertEqual(source_route(t,{})['route'],'candidates_then_room')
 def test_explicit_source_takes_precedence(self):
  t=self.parse(2,'Go to the living room, grasp the canned_juice on the wooden_side_table and throw it in the trash_box_for_bottle_can.')
  self.assertEqual(source_route(t,{'complete':True,'objects':[]})['furniture_ids'],['wooden_side_table#living_room'])
 def session(self):
  s=DynamicSession('s',profile={'layout':'LayoutA'});s.preflight(ready=True);s.moderator('Environment','LayoutA',session_id='s');s.moderator('Are_you_ready?',session_id='s');s.drain();return s
 def result(self,s,**result):s.result(s.pending,dict(status='ok',**result),session_id=s.session_id,run_id=s.run_id)
 def test_absence_then_corrected_instruction_and_fresh_binding(self):
  s=self.session();text='Go to the lobby, grasp the bear_doll and give it to me.';s.moderator('Instruction',text,session_id='s')
  t=self.parse(3,text);t['destination']['room']='lobby';self.result(s,task=t)
  self.result(s,room='lobby',arrived=True);self.assertEqual(s.state,'locate_source');s.drain()
  self.result(s,target='bear_doll',room='lobby',confirmed=False,search_exhausted=True,valid_views=24,required_views=24,unresolved_candidates=0,evidence_ref='live-search')
  self.assertEqual(s.state,'waiting_correction');self.assertEqual(s.drain()[0]['message'],'Does_not_exist')
  s.moderator('Corrected_instruction','Go to the lobby, grasp the dog_doll and give it to me.',session_id='s')
  self.assertEqual(s.state,'resolve_task');self.assertEqual(s.revision,2);self.assertEqual(s.evidence,{})
 def test_incomplete_search_cannot_report_absence(self):
  s=self.session();text='Go to the lobby, grasp the bear_doll and give it to me.';s.moderator('Instruction',text,session_id='s');t=self.parse(3,text);t['destination']['room']='lobby';self.result(s,task=t);self.result(s,room='lobby',arrived=True);s.drain()
  self.result(s,target='bear_doll',room='lobby',confirmed=False,search_exhausted=True,valid_views=23,required_views=24,unresolved_candidates=0,evidence_ref='partial')
  self.assertFalse(any(e.get('message')=='Does_not_exist' for e in s.drain()))
 def test_changed_object_complete_table_task(self):
  s=self.session();text='Go to the living room, grasp the apple and put it on the square_low_table.';s.moderator('Instruction',text,session_id='s');t=self.parse(1,text);self.result(s,task=t)
  self.result(s,room='living_room',arrived=True)
  self.result(s,target='apple',room='living_room',confirmed=True,source_table='white_side_table#living_room',evidence_ref='visual')
  self.result(s,target='apple',room='living_room',confirmed=True,source_table='white_side_table#living_room',evidence_ref='near')
  self.result(s,target='apple',grasp_verified=True);self.result(s,holding_verified=True,carry_ready=True)
  self.result(s,destination='square_low_table#living_room',arrived=True);self.result(s,release_sent=True)
  self.result(s,target='apple',destination='square_low_table#living_room',release_sent=True,completion_actions_confirmed=True,basis='gripper_release_completed')
  self.assertEqual(s.state,'waiting_moderator');s.moderator('Task_succeeded',session_id='s');self.assertEqual(s.state,'succeeded')

class DynamicBindingTests(unittest.TestCase):
 def bundle(self,n):
  from types import SimpleNamespace
  from unittest.mock import Mock
  from task_generalization import compile_all
  cfg=compile_all(ROOT,json.loads((ROOT/'competition_tasks.json').read_text()))[config_name(n)]
  env=yaml.safe_load(Path(cfg['environment']).read_text());sem=json.loads(Path(cfg['semantic_map']).read_text())
  return {'config':cfg,'environment':env,'semantic':sem,'runtime':Mock(),'grasp_backend':Mock(),'motion_backend':Mock(),'worker':Mock(),'output':None},SimpleNamespace(config=cfg,check=lambda c:None)
 def test_all_default_tasks_resolve_in_live_adapter_without_motion(self):
  from dynamic_bindings import DynamicBindings
  from tempfile import TemporaryDirectory
  doc=json.loads((ROOT/'competition_tasks.json').read_text())
  for spec in doc['sessions']:
   with TemporaryDirectory() as tmp:
    b,s=self.bundle(spec['session_number']);b['output']=s.output=Path(tmp)
    adapter=DynamicBindings(b,s)
    result=adapter.resolve({'instruction':spec['expected_instruction'],'task_revision':1},None)
    self.assertEqual(result['task']['target'],spec.get('initial_target',spec['target']))
    b['runtime'].move_joints.assert_not_called()
    b['motion_backend'].navigate.assert_not_called()
 def test_static_absence_skips_candidate_furniture_but_runs_room_scan(self):
  from dynamic_bindings import DynamicBindings
  from types import SimpleNamespace
  from unittest.mock import Mock
  a=DynamicBindings.__new__(DynamicBindings);a.route={'route':'room_search','furniture_ids':[]};a.stages=SimpleNamespace(check=lambda c:None,motion=SimpleNamespace());a.candidate_ids=Mock(side_effect=AssertionError('must skip furniture'))
  a.search=Mock();a.search.search.return_value={'status':'ok','confirmed':False,'search_exhausted':True}
  self.assertTrue(a.locate({},None)['search_exhausted']);a.search.search.assert_called_once()
 def test_binding_miss_tries_remaining_candidates_then_room(self):
  from dynamic_bindings import DynamicBindings
  from types import SimpleNamespace
  from unittest.mock import Mock
  a=DynamicBindings.__new__(DynamicBindings);a.route={'route':'bound_then_candidates_then_room','furniture_ids':['first']};a.stages=SimpleNamespace(check=lambda c:None,motion=SimpleNamespace());a.candidate_ids=lambda:['first','second'];a.confirm_furniture=Mock(return_value=None)
  a.search=Mock();a.search.search.return_value={'status':'ok','confirmed':False,'search_exhausted':True}
  a.locate({},None)
  self.assertEqual([x.args[0] for x in a.confirm_furniture.call_args_list],['first','second']);a.search.search.assert_called_once()

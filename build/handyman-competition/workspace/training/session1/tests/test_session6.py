import unittest,copy
from prepare_session6_config import build,profile,INSTRUCTION
from live_config import validate
from session1 import Session1
from run_sessions12 import round_spec
class Session6Tests(unittest.TestCase):
 def test_exact_official_instruction(self):
  p=profile('dining_table#lobby');s=Session1('s6',profile=p)
  self.assertEqual(s.instruction,INSTRUCTION);self.assertEqual(s.task.target,'filled_ketchup')
  self.assertEqual(s.task.destination_room,'living_room');self.assertEqual(round_spec(5)['layout'],'LayoutD')
 def test_source_is_required_and_must_resolve(self):
  with self.assertRaises(ValueError):profile('body#kitchen')
  source='dining_table#lobby';sem={'destinations':[dict(id=source,room='lobby'),dict(id='wooden_shelf#living_room',room='living_room')]}
  env={'destinations':{'dining_table':[dict(room='lobby',x=1,y=2,yaw=0)],'wooden_shelf':[dict(room='living_room',x=3,y=4,yaw=0)]}}
  c=build({},source,sem,env,'env.yaml');validate(c)
  self.assertEqual(c['source_table_id'],source)
  bad=copy.deepcopy(c);bad['task']['object']='rabbit_doll'
  with self.assertRaises(ValueError):validate(bad)
  with self.assertRaises(ValueError):build({},'armchair#lobby',sem,env,'env.yaml')

import unittest,json,copy
from pathlib import Path
from session1 import Session1
from live_config import validate
from prepare_session5_config import build
from run_sessions12 import round_spec
class Session5Tests(unittest.TestCase):
 def setUp(self):
  self.profile=json.loads((Path(__file__).parents[1]/'profiles/session5.json').read_text())
 def test_profile_instruction_and_cross_room(self):
  s=Session1('test',profile=self.profile)
  self.assertEqual(s.task.room,'kitchen');self.assertEqual(s.task.destination_room,'living_room')
  self.assertEqual(s.instruction,'Go to the kitchen, grasp the rabbit_doll and put it on the square_low_table.')
  self.assertEqual(round_spec(4)['layout'],'LayoutD')
 def test_config_resolves_exact_source_and_clears_container(self):
  sem={'destinations':[{'id':'body#kitchen','room':'kitchen'},{'id':'square_low_table#living_room','room':'living_room'}]}
  env={'destinations':{'body':[dict(room='kitchen',x=1,y=2,yaw=0)],'square_low_table':[dict(room='living_room',x=3,y=4,yaw=0)]}}
  c=build({'container_opening':{}},self.profile,sem,env,'/tmp/env.yaml');validate(c)
  self.assertNotIn('container_opening',c);self.assertEqual(c['destination_room'],'living_room')
  bad=copy.deepcopy(c);bad['session_profile']['source']['furniture_id']='body#lobby'
  with self.assertRaises(ValueError):validate(bad)

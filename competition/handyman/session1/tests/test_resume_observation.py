import unittest,tempfile,json
from pathlib import Path
from session1 import Session1,INSTRUCTION
from resume_observation import restore
class ResumeTests(unittest.TestCase):
 def test_resume_requires_received_task_and_reobserves_without_protocol(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);(root/'task').mkdir();nav=root/'nav.json';nav.write_text(json.dumps({'status':'arrived','nav_reset':True}))
   session=Session1('new',source_table='white_side_table#living_room');saved=session.snapshot();saved.update(state='stopped',evidence={'navigate_room':{'arrived':True},'navigate_source':{'arrived':True,'evidence_ref':str(nav)}})
   checkpoint=root/'task/live-state.json';checkpoint.write_text(json.dumps(saved));(root/'live.log').write_text('Moderator: Instruction '+INSTRUCTION)
   env={'destinations':{'white_side_table':[{'room':'living_room','x':1,'y':2,'yaw':0}]}}
   restore(session,checkpoint,{'poses':{'map':[1,2,0]}},env)
   effects=session.drain();self.assertEqual(len(effects),1);self.assertEqual(effects[0]['stage'],'observe_target')
   (root/'live.log').write_text('')
   with self.assertRaisesRegex(ValueError,'actual_received'):restore(session,checkpoint,{'poses':{'map':[1,2,0]}},env)

import unittest,json
from pathlib import Path
from prepare_session2_config import build
class Session2ConfigTests(unittest.TestCase):
    def test_build_does_not_mutate_successful_session1(self):
        p=json.loads((Path(__file__).resolve().parents[1]/'profiles/session2.json').read_text())
        ids=[p[r]['furniture_id'] for r in ['source','destination']]
        semantic={'destinations':[dict(id=i,room='living_room') for i in ids]}
        env={'destinations':{i.split('#')[0]:[dict(room='living_room',x=0,y=0,yaw=0)] for i in ids}}
        base={'layout':'LayoutA','task':{'object':'dog_doll'}}
        opening=dict(destination_id=ids[1],rim_height_m=.5,polygon_map_m=[[0,0],[1,0],[1,1],[0,1]])
        c=build(base,p,semantic,env,'layout_b.yaml',opening)
        self.assertEqual(base['layout'],'LayoutA');self.assertEqual(c['task']['object'],'canned_juice')
        self.assertEqual(c['session_profile']['destination']['mode'],'container_drop')

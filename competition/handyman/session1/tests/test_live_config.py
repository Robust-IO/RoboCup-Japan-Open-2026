import unittest,json
from pathlib import Path
from live_config import validate
class ConfigTests(unittest.TestCase):
    def config(self):
        p=json.loads((Path(__file__).resolve().parents[1]/'profiles/session2.json').read_text())
        return dict(layout='LayoutB',task={'object':'canned_juice'},source_table_id=p['source']['furniture_id'],destination_id=p['destination']['furniture_id'],session_profile=p,container_opening={'destination_id':p['destination']['furniture_id']})
    def test_session2_matching(self):self.assertEqual(validate(self.config())['layout'],'LayoutB')
    def test_wrong_room_not_silently_substituted(self):
        c=self.config();c['destination_id']='trash_box_for_bottle_can#kitchen'
        with self.assertRaises(ValueError):validate(c)

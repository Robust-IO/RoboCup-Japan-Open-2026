import unittest,json
from pathlib import Path
from prepare_session4_config import build
from live_config import validate,require_container_execution_ready
from session1 import Session1

class Session4Tests(unittest.TestCase):
    def profile(self):return json.loads((Path(__file__).resolve().parents[1]/'profiles/session4.json').read_text())
    def config(self,opening=None):
        p=self.profile();ids=[p[r]['furniture_id'] for r in ('source','destination')]
        semantic={'destinations':[dict(id=i,room='lobby') for i in ids]}
        env={'destinations':{i.split('#')[0]:[dict(room='lobby',x=0,y=0,yaw=0)] for i in ids}}
        return build({'container_opening':{'destination_id':'OLD_BIN'},'layout':'LayoutB'},p,semantic,env,'layout_c.yaml',opening)
    def test_pending_geometry_cannot_execute_or_reuse_session2(self):
        c=self.config();validate(c)
        self.assertNotIn('polygon_map_m',c['container_opening'])
        with self.assertRaises(ValueError):require_container_execution_ready(c)
    def test_reviewed_session4_opening_enables_execution(self):
        c=self.config(dict(destination_id='trash_box_for_burnable#lobby',rim_height_m=.6,polygon_map_m=[[0,0],[1,0],[1,1],[0,1]]))
        validate(c);require_container_execution_ready(c)
        self.assertEqual(c['source_table_id'],'white_side_table#lobby')
        self.assertEqual(c['task']['object'],'tumbler')
    def test_wrong_bin_rejected(self):
        with self.assertRaises(ValueError):self.config({'destination_id':'trash_box_for_bottle_can#living_room'})
    def test_official_instruction_uses_shared_container_stages(self):
        p=self.profile();s=Session1('s4',profile=p);s.preflight(ready=True)
        s.moderator('Environment','LayoutC',session_id='s4');s.moderator('Are_you_ready?',session_id='s4');s.drain()
        s.moderator('Instruction',p['expected_instruction'],session_id='s4')
        self.assertEqual(s.state,'navigate_room');self.assertEqual(s.task.source_table,'white_side_table#lobby')
        self.assertEqual(s.task.destination,'trash_box_for_burnable#lobby');self.assertIn('verify_placement',s.steps)
    def test_session4_entry_is_in_supported_sequence(self):
        from round_sequence import RoundSequence
        from run_sessions12 import round_spec
        self.assertEqual(len(RoundSequence.layouts),6)
        self.assertEqual(round_spec(3)['layout'],'LayoutC')
        self.assertEqual(round_spec(3)['runner'].name,'run_live_session1.py')

class RimMeasurementTests(unittest.TestCase):
    def test_measures_reviewed_geometry_without_colour_filter(self):
        import tempfile,numpy as np
        from types import SimpleNamespace
        from unittest.mock import patch
        from measure_container_rim import measure
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);tf=dict(translation=[0,0,0],quaternion=[0,0,0,1])
            s=dict(streams={'rgb':{'stamp_ns':10},'depth':{'stamp_ns':10}},rgb_k=[100,0,10,0,100,10,0,0,1],base_from_rgb=tf,tracking_transforms={'map_from_base_at_depth':tf})
            (root/'snapshot.json').write_text(json.dumps(s));depth=np.ones((20,20));np.save(root/'depth_m.npy',depth)
            with patch.dict('sys.modules',{'hand_camera_ray':SimpleNamespace(rotation=lambda q:np.eye(3))}):
                result=measure(root,[[5,5],[15,5],[15,15],[5,15]],[[4,4],[10,4],[16,4]],'trash_box_for_burnable#lobby')
                self.assertEqual(result['rim_height_m'],1.);self.assertFalse(result['object_truth_coordinates_used'])
                self.assertEqual(result['polygon_map_m'][0],[-.05,-.05])
                depth[5,5]=0;np.save(root/'depth_m.npy',depth)
                with self.assertRaises(ValueError):measure(root,[[5,5],[15,5],[15,15],[5,15]],[[4,4],[10,4],[16,4]],'trash_box_for_burnable#lobby')

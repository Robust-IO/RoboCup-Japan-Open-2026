import sys
from pathlib import Path
import tempfile
import unittest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from single_search_request import request_body,selected_point


class Tests(unittest.TestCase):
    def test_pinned_identity_and_no_motion_authority(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);path=root/'config/environments';path.mkdir(parents=True)
            env=dict(internal_name='LayoutA',map='package://handyman_rebuild_ros2/maps/LayoutA/map.yaml',
                rooms=dict(living_room=dict(region=[[0,0],[5,0],[5,5],[0,5]],
                    search_points=[dict(x=i+1,y=1,yaw=0) for i in range(3)])))
            (path/'layout_a.yaml').write_text(yaml.safe_dump(env))
            r=request_body(root,'t')
            self.assertEqual(r['points'][2]['id'],'LayoutA/living_room/2')
            self.assertEqual(r['target'],'canned_juice');self.assertFalse(r['actionable'])
            self.assertNotIn('stamp_ns',r)
    def test_missing_layout_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(FileNotFoundError):request_body(Path(d),'t')
    def test_select_zero(self):
        body={'points':[{'id':'zero'},{'id':'one'}]}
        self.assertEqual(selected_point(body,0)['id'],'zero')
        self.assertEqual(selected_point(body,1)['id'],'one')
    def test_bad_selection_rejected(self):
        for index in (-1,2,True,0.0,'0'):
            with self.subTest(index=index),self.assertRaises(ValueError):
                selected_point({'points':[{},{}]},index)
    def test_empty_rejected(self):
        with self.assertRaises(ValueError):selected_point({'points':[]},0)


if __name__=='__main__':unittest.main()

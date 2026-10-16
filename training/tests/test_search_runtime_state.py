import copy
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_runtime_state import ObserverRecords

class RecordsTest(unittest.TestCase):
    def setUp(self):
        self.records=ObserverRecords('a'*32,'LayoutA/kitchen/0','b'*64,'apple')
        self.row=dict(task_id='a'*32,target='apple',observer_context=dict(self.records.context),
            actionable=False,does_not_exist_authorized=False,terminal=True,state='found',
            position_m=[1.,2.,3.],position_frame='odom')
    def test_found(self):
        self.records.accept(self.row);self.assertEqual(self.records.terminal['state'],'found')
    def test_wrong_identity(self):
        for key,value in [('task_id','c'*32),('target','canned_juice'),('actionable',True),('does_not_exist_authorized',True)]:
            with self.assertRaises(ValueError):self.records.accept(dict(self.row,**{key:value}))
    def test_old_point(self):
        row=copy.deepcopy(self.row);row['observer_context']['point_id']='LayoutA/kitchen/1'
        with self.assertRaises(ValueError):self.records.accept(row)
    def test_bad_position(self):
        for p in ([1,2,float('nan')],[1,2],[True,2,3],None):
            with self.assertRaises(ValueError):self.records.accept(dict(self.row,position_m=p))
    def test_no_terminal_on_zero_exit(self):
        class Process:
            def poll(self):return 0
        self.assertFalse(self.records.finished(Process()))
    def test_no_rewrite(self):
        self.records.accept(self.row)
        with self.assertRaises(ValueError):self.records.accept(self.row)

if __name__=='__main__':unittest.main()

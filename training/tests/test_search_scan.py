import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_scan import SearchScan

class SearchTests(unittest.TestCase):
    def make(self): return SearchScan('t1','canned_juice',[dict(x=0,y=0,yaw=0)],0)
    def ready(self):
        s=self.make(); s.arrived('t1:0',True,True,1); return s
    def test_found(self):
        s=self.ready(); s.observe('t1:0','canned_juice',1,2,True,True,[1,2,3])
        self.assertEqual(s.phase,'found'); self.assertFalse(s.status()['actionable'])
    def test_absence_not_protocol_absence(self):
        s=self.ready()
        for i in range(1,4): s.observe('t1:0','canned_juice',i,2+i*.1,True,absent=True)
        self.assertEqual(s.phase,'plan_exhausted'); self.assertFalse(s.status()['does_not_exist_authorized'])
    def test_failure_not_absence(self):
        s=self.make(); s.arrived('t1:0',False,False,1)
        self.assertEqual(s.phase,'incomplete')
    def test_unhealthy_not_negative(self):
        s=self.ready()
        for i in range(1,5): s.observe('t1:0','canned_juice',i,2,False,absent=True)
        self.assertEqual(s.negative,0); s.tick(7); self.assertEqual(s.phase,'incomplete')
    def test_duplicate(self):
        s=self.ready()
        for _ in range(4): s.observe('t1:0','canned_juice',1,2,True,absent=True)
        self.assertEqual(s.phase,'observe')
    def test_wrong_task_view_target(self):
        s=self.ready(); s.observe('old:0','canned_juice',1,2,True,True,[1,2,3])
        s.observe('t1:0','apple',2,2,True,True,[1,2,3]); self.assertEqual(s.phase,'observe')
    def test_cancel(self):
        s=self.ready(); s.cancel(); s.observe('t1:0','canned_juice',1,2,True,True,[1,2,3])
        self.assertEqual(s.phase,'cancelled')
    def test_deadline(self):
        s=self.ready(); s.observe('t1:0','canned_juice',1,61,True,True,[1,2,3])
        self.assertEqual(s.phase,'incomplete')
    def test_not_settled(self):
        s=self.make(); s.arrived('t1:0',True,False,1); self.assertEqual(s.phase,'navigate')

if __name__=='__main__': unittest.main()

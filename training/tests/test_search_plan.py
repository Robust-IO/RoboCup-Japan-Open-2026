import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from build_search_plan import build,inside

class PlanTests(unittest.TestCase):
    def config(self,points): return {'internal_name':'LayoutA','rooms':{'room':{'region':[[0,0],[2,0],[2,2],[0,2]],'search_points':points}}}
    def test_valid(self):
        p=build(self.config([dict(x=1,y=1,yaw=0)]))
        self.assertTrue(p['rooms'][0]['valid']); self.assertFalse(p['coverage_verified'])
    def test_outside(self): self.assertFalse(build(self.config([dict(x=3,y=1,yaw=0)]))['rooms'][0]['valid'])
    def test_nan(self): self.assertFalse(build(self.config([dict(x=float('nan'),y=1,yaw=0)]))['rooms'][0]['valid'])
    def test_empty(self): self.assertFalse(build(self.config([]))['rooms'][0]['valid'])
    def test_duplicate(self):
        p=dict(x=1,y=1,yaw=0)
        self.assertFalse(build(self.config([p,p]))['rooms'][0]['valid'])
    def test_boundary(self): self.assertTrue(inside(0,1,[(0,0),(2,0),(2,2),(0,2)]))

if __name__=='__main__': unittest.main()

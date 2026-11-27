import ast,unittest,math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
class StagingFallbackTests(unittest.TestCase):
 def options(self,selected):
  tree=ast.parse((Path(__file__).parents[1]/'adaptive_grasp_staging.py').read_text())
  prep=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='prepare')
  node=next(n for n in prep.body if isinstance(n,ast.FunctionDef) and n.name=='options')
  env=dict(normal=[[1,0,0],[2,0,0],[3,0,0]],ranked=[[1,0,0],[2,0,0],[3,0,0]],scene=None,start=[0,0,0],backend=SimpleNamespace(table={}),stages=SimpleNamespace(target='filled_ketchup',source_id='table'),
    transform_scene=lambda scene,start,pose:pose,approach_plan=lambda pose,*a:{'selected':selected(pose)},is_cabinet=lambda t:True,np=np,math=math)
  exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual staging options>','exec'),env)
  return list(env['options']())
 def test_all_geometry_rejected_still_visits_ranked_viewpoints(self):
  rows=self.options(lambda p:None);self.assertEqual([r[0][0] for r in rows],[1,2,3]);self.assertTrue(all('fresh_observation' in r[2] for r in rows))
 def test_feasible_preferred_without_dropping_other_views(self):
  rows=self.options(lambda p:{} if p[0]==2 else None);self.assertEqual([r[0][0] for r in rows],[2,1,3])

class RejectedNormalOffsetTests(unittest.TestCase):
 def run_options(self,accept,cabinet=False):
  tree=ast.parse((Path(__file__).parents[1]/'adaptive_grasp_staging.py').read_text())
  prep=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='prepare')
  node=next(n for n in prep.body if isinstance(n,ast.FunctionDef) and n.name=='options')
  calls=[]
  def rank(start,poses,source):
   calls.extend(poses)
   return ([p for p in poses if accept(p)],{})
  env=dict(normal=[[1.,0.,0.]],ranked=[],scene=None,start=[0.,0.,0.],target=[2.,0.,.5],
   backend=SimpleNamespace(table={},record=lambda e:None),stages=SimpleNamespace(target='dog_doll',source_id='sofa'),
   selector=SimpleNamespace(rank=rank),polygon=[],inside=lambda *a:True,
   transform_scene=lambda scene,start,pose:pose,approach_plan=lambda *a:{'selected':{}},
   is_cabinet=lambda t:cabinet,obstacle_distance=lambda scene,p,start:abs(p[1]),np=np,math=math)
  exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual options>', 'exec'),env)
  return list(env['options']()),calls
 def test_unreachable_normal_still_checks_both_sides(self):
  rows,calls=self.run_options(lambda p:p[1]>0)
  self.assertEqual(len(calls),6)
  self.assertEqual(len(rows),3)
  self.assertTrue(all(p[1]>0 and basis=='obstacle_offset' for p,a,basis in rows))
  self.assertTrue(all(0<abs(p[2])<=math.radians(15) for p,a,basis in rows))
 def test_unreachable_offsets_never_become_goals(self):
  rows,calls=self.run_options(lambda p:False)
  self.assertEqual(rows,[]);self.assertEqual(len(calls),6)
 def test_cabinet_keeps_front_alignment_rule(self):
  rows,calls=self.run_options(lambda p:True,cabinet=True)
  self.assertEqual(rows,[]);self.assertEqual(calls,[])

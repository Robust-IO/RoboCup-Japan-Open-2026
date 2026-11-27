import ast,json,unittest
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
class ScopeTests(unittest.TestCase):
 def test_trial_feature_follows_target_in_any_session(self):
  from object_grasp_profiles import apply
  for i in range(1,7):
   filename='session3-search-config.json' if i==3 else f'session{i}-live-config.json'
   cfg=json.loads((ROOT/filename).read_text());backend=SimpleNamespace()
   apply(backend,cfg['task']['object'])
   self.assertEqual(backend.trial_before_rotation_enabled,cfg['task']['object']=='filled_ketchup')
   apply(backend,'filled_ketchup');self.assertTrue(backend.trial_before_rotation_enabled)
   apply(backend,'canned_juice');self.assertFalse(backend.trial_before_rotation_enabled)
 def test_disabled_backend_does_not_observe_or_move(self):
  tree=ast.parse((ROOT/'grasp_runtime/autograsp/ros_backend.py').read_text())
  method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='try_aligned_trial_before_rotation')
  ns={};exec(compile(ast.Module(body=[method],type_ignores=[]),'<method>','exec'),ns)
  self.assertFalse(ns[method.name](SimpleNamespace(),'contact_band_unobserved'))

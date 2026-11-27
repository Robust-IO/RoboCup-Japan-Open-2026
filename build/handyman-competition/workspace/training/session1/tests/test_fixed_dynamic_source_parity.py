import unittest,json,tempfile
from pathlib import Path
from types import SimpleNamespace
from copy import deepcopy
import yaml
from stage_bindings import StageBindings
from dynamic_bindings import DynamicBindings
from task_generalization import config_name
ROOT=Path(__file__).resolve().parents[4]
class SourceParityTests(unittest.TestCase):
 def test_all_six_fixed_and_dynamic_candidates_match(self):
  for number in range(1,7):
   with self.subTest(session=number),tempfile.TemporaryDirectory() as out:
    cfg=json.loads((ROOT/'workspace/training/session1'/config_name(number)).read_text())
    env=yaml.safe_load(Path(cfg['environment']).read_text())
    sem=json.loads(Path(cfg['semantic_map']).read_text())
    table=next(t for t in sem['destinations'] if t['id']==cfg['source_table_id'])
    backend=SimpleNamespace(table=table)
    fixed=StageBindings(None,None,backend,cfg,env,out,map_point=lambda *a:None)
    dyn=DynamicBindings.__new__(DynamicBindings)
    dyn.cfg=deepcopy(cfg);dyn.b={'semantic':sem,'environment':env};dyn.backend=SimpleNamespace()
    dyn.stages=SimpleNamespace()
    dyn.set_source(table['id'])
    self.assertEqual(fixed.source_candidates,dyn.stages.source_candidates)
    self.assertEqual(fixed.source_goal,dyn.stages.source_goal)
    self.assertEqual(fixed.source_room,dyn.stages.source_room)
    if number==3:self.assertGreater(len(fixed.source_candidates),12)
 def test_dynamic_uses_shared_navigation_and_grasp_handlers(self):
  from unittest.mock import Mock
  b=DynamicBindings.__new__(DynamicBindings);b.stages=SimpleNamespace(grasp=Mock())
  self.assertIs(b.handlers()['grasp'],b.stages.grasp)

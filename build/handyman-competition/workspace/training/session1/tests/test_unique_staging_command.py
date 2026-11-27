import ast,unittest,tempfile
from pathlib import Path
from types import SimpleNamespace
class UniqueStagingTests(unittest.TestCase):
 def test_each_staging_round_gets_unique_operation(self):
  import uuid
  source=(Path(__file__).parents[1]/'adaptive_grasp_staging.py').read_text()
  tree=ast.parse(source);node=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='staging_round' for t in n.targets))
  values=[]
  for _ in range(2):
   env={'uuid':uuid};exec(compile(ast.Module(body=[node],type_ignores=[]),'<round>','exec'),env);values.append(env['staging_round'])
  self.assertNotEqual(*values);self.assertIn("f'-grasp-staging-{staging_round}-{index}'",source)
 def test_existing_result_prevents_child_start_or_old_result_read(self):
  source=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/apple_transfer_auto_backend.py').read_text()
  node=next(n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef) and n.name=='command')
  env={'Path':Path};exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual command>','exec'),env)
  with tempfile.TemporaryDirectory() as d:
   result=Path(d)/'old.json';result.write_text('{"status":"arrived"}')
   with self.assertRaisesRegex(ValueError,'result_path_already_exists'):env['command'](SimpleNamespace(),[],result)
   self.assertEqual(result.read_text(),'{"status":"arrived"}')

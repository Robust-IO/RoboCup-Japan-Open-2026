import ast,math,unittest
from pathlib import Path
from types import SimpleNamespace
from transfer_actions import tuck
class SynchronousTuckTests(unittest.TestCase):
 def runtime(self):
  rt=SimpleNamespace(joints=[.6,-1.7,.1,.13,.3,.21],calls=[])
  rt.state=lambda:dict(joints=list(rt.joints))
  rt.sample=lambda **kw:None
  def move(goal,label):rt.calls.append((label,list(goal)));rt.joints=list(goal)
  rt.move_joints=move
  return rt
 def test_withdrawal_precedes_single_held_trajectory(self):
  rt=self.runtime();order=[]
  def withdraw():
   self.assertFalse(rt.calls);order.append('withdraw');return dict(clear_to_fold=True)
  def observe(label):order.append(label);return dict(holding_verified=True)
  tuck(rt,observe_hold=observe,withdraw_from_source=withdraw)
  self.assertEqual(len(rt.calls),1);self.assertEqual(rt.joints,[0.,0.,0.,-math.pi/2,0.,.21])
  self.assertLess(order.index('withdraw'),order.index('after-tuck'))
 def test_shared_navigation_preparation_uses_one_trajectory(self):
  path=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/outcome_transfer_runtime.py')
  tree=ast.parse(path.read_text());node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='prepare_navigation_from_current')
  ns={};exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),ns)
  rt=self.runtime();ns[node.name](rt)
  self.assertEqual(len(rt.calls),1);self.assertEqual(rt.joints,[0.,0.,0.,-math.pi/2,0.,.21])
 def test_empty_cleanup_uses_one_trajectory(self):
  path=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/table_transfer_core.py');tree=ast.parse(path.read_text())
  node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='fold_after_withdrawal')
  ns=dict(math=math,_state=lambda rt:rt.state()['joints']);exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),ns)
  rt=self.runtime();rt.joints[5]=.65;ns[node.name](rt)
  self.assertEqual(len(rt.calls),1);self.assertEqual(rt.joints,[0.,0.,0.,-math.pi/2,0.,.65])

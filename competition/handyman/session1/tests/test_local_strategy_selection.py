import unittest
from types import SimpleNamespace
from grasp_runtime.autograsp.planner import Policy
from grasp_runtime.autograsp.local_strategy_selection import choose
class LocalStrategyTests(unittest.TestCase):
 def run_case(self,side,top,filter_side=False):
  b=SimpleNamespace();calls=[]
  def planner(s,h,p):
   calls.append(p.preferred_strategy);return {'selected':side if p.preferred_strategy=='side' else top,'rejected':{}}
  if filter_side:b.filter_approach_decision=lambda d:dict(d,selected=None) if d['selected']==side else d
  result=choose(b,None,None,Policy(),{'overhead_after_fold':True,'height_to_width_ratio':.36},allow_overhead=True,planner=planner)
  return b,calls,result
 def test_ratio_cannot_override_feasible_side(self):
  b,c,r=self.run_case({'strategy':'side'},{'strategy':'overhead'});self.assertEqual(c,['side']);self.assertEqual(r[0].allowed_strategies,('side',))
 def test_blocked_side_can_try_overhead(self):
  b,c,r=self.run_case({'strategy':'side'},{'strategy':'overhead'},True);self.assertEqual(c,['side','overhead']);self.assertEqual(r[0].allowed_strategies,('overhead',))
 def test_no_candidate_does_not_latch_overhead(self):
  b,c,r=self.run_case(None,None);self.assertEqual(b.policy.allowed_strategies,('side',));self.assertIsNone(r[1]['selected'])

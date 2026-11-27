import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from handover_reclose import confirm
from grasp_runtime.autograsp.coordinator import AttemptFailure
class RecloseTests(unittest.TestCase):
 def run_case(self,opening):
  ticks=[0];stamp=[0];pub=Mock();node=Mock()
  def clock():ticks[0]+=.1;return ticks[0]
  def state():
   stamp[0]+=100000000
   return {'stamp_ns':stamp[0],'joints':[0,0,0,0,0,opening]}
  rt=SimpleNamespace(publisher=lambda t:pub,node=node,ownership=lambda t:None,trajectory=lambda names,positions,duration:positions,pump=lambda dt:None,state=state)
  try:
   with patch.dict('sys.modules',{'canned_grasp_runtime':SimpleNamespace(TOPICS=['a','b','c','hand'])}):
    return confirm(rt,lambda:None,lambda e:None,clock=clock)
  finally:
   pub.publish.assert_called_once_with([0.]);node.destroy_publisher.assert_called_once_with(pub)
 def test_blocked_jaw_confirms(self):self.assertEqual(self.run_case(.35),.35)
 def test_empty_jaw_rejects(self):
  with self.assertRaises(AttemptFailure):self.run_case(0.)

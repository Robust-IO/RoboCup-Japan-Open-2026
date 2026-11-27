import unittest,tempfile
from pathlib import Path
from types import SimpleNamespace
from threading import Event
import numpy as np
from candidate_selection import CandidateSelector
from handover_live import HandoverLive
class HandoverApproachTests(unittest.TestCase):
 def test_no_exported_staging_required_and_current_side_retained(self):
  for start in ([1.,3.,0.],[5.,3.,1.],[3.,1.,-1.]):
   with tempfile.TemporaryDirectory() as tmp:
    state={'poses':{'map':start}}
    calls=[]
    def navigate(*args):calls.append(args);return {'arrived':True}
    stages=SimpleNamespace(runtime=SimpleNamespace(sample=lambda **k:None,state=lambda:state),
      motion=SimpleNamespace(),grasp_backend=None,output=Path(tmp),
      config={'handover':{'delivery_point_map_m':[3,3,1]}},check=lambda c:None,
      candidate_selector=CandidateSelector(np.ones((120,120),bool),.05,[0,0,0],.26,{}),
      environment={'rooms':{'lobby':{'region':[[0,0],[6,0],[6,6],[0,6]]}}},navigate=navigate)
    h=HandoverLive(stages,map_point=lambda *a:None,rotation=lambda *a:None)
    self.assertTrue(h.navigate({'stage':'navigate_moderator'},Event())['arrived'])
    self.assertEqual(len(calls),1)
    self.assertLess(np.linalg.norm(np.array(calls[0][2][:2])-start[:2]),1.3)

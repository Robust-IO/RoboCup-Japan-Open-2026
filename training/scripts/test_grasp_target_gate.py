from dataclasses import replace
import unittest
from grasp_target_gate import TargetEvidence, check_target


class GateTests(unittest.TestCase):
    def setUp(self):
        self.good=TargetEvidence('task-1','apple','base_link',10.,(.6,.1,.7),.9,True,True)

    def check(self,e):
        return check_target(e,task_id='task-1',class_name='apple',now=10.1)[0]

    def test_planning_candidate(self): self.assertTrue(self.check(self.good))
    def test_zero_depth(self): self.assertFalse(self.check(replace(self.good,measured_depth_m=0)))
    def test_invalid_depth(self): self.assertFalse(self.check(replace(self.good,depth_valid=False)))
    def test_stale(self): self.assertFalse(self.check(replace(self.good,stamp=8.)))
    def test_clock_reset(self): self.assertFalse(self.check(replace(self.good,stamp=11.)))
    def test_wrong_task(self): self.assertFalse(self.check(replace(self.good,task_id='old-task')))
    def test_wrong_class(self): self.assertFalse(self.check(replace(self.good,class_name='sugar')))
    def test_frame(self): self.assertFalse(self.check(replace(self.good,frame_id='map')))
    def test_nonfinite(self): self.assertFalse(self.check(replace(self.good,point=(float('nan'),0,0))))
    def test_room(self): self.assertFalse(self.check(replace(self.good,room_confirmed=False)))


if __name__=='__main__': unittest.main()

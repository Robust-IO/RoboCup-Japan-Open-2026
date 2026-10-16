import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rgbd_quality import StabilityGate, transform_point

def observation(i=1,**kw):
    o=dict(alignment_verified=True,optical_frame_verified=True,instances=1,class_conflict=False,
           confidence=.99,rgb_stamp_ns=i*1_000_000_000,depth_stamp_ns=i*1_000_000_000,
           tf_stamp_ns=i*1_000_000_000,core_pixels=100,valid_pixels=100,
           depth_p05_median_p95=[1.73,1.74,1.75],position_m=[1.7,.1,.6],target='canned_juice',frame_id='odom')
    o.update(kw)
    return o

class QualityTests(unittest.TestCase):
    def test_stable_unique(self):
        gate=StabilityGate()
        for i in range(1,4):
            result=gate.update(observation(i),i*1_000_000_000)
            self.assertEqual(result['stable'],i==3)
    def test_reject_conditions(self):
        cases=[dict(alignment_verified=False),dict(optical_frame_verified=False),dict(instances=2),
               dict(class_conflict=True),dict(confidence=.49),dict(depth_stamp_ns=1_080_000_000),
               dict(tf_stamp_ns=0),dict(core_pixels=0),dict(valid_pixels=50),
               dict(depth_p05_median_p95=[1.,2.,3.]),dict(position_m=[float('nan'),0,0])]
        for kw in cases:
            with self.subTest(kw=kw):
                gate=StabilityGate()
                self.assertEqual(gate.update(observation(**kw),1_100_000_000)['status'],'rejected')
                self.assertFalse(gate.history)
    def test_duplicate_resets(self):
        gate=StabilityGate(); gate.update(observation(),1_000_000_000)
        self.assertEqual(gate.update(observation(),1_000_000_000)['reason'],'duplicate_or_reversed_time')
    def test_jump_resets(self):
        gate=StabilityGate(); gate.update(observation(),1_000_000_000)
        self.assertEqual(gate.update(observation(2,position_m=[2.,0.,0.]),2_000_000_000)['count'],1)
    def test_old_future_missing(self):
        for now in [0,4_000_000_000]:
            self.assertFalse(StabilityGate().update(observation(),now)['stable'])
        self.assertEqual(StabilityGate().update({},0)['reason'],'malformed_observation')
    def test_target_change(self):
        gate=StabilityGate(); gate.update(observation(),1_000_000_000)
        self.assertEqual(gate.update(observation(2,target='apple'),2_000_000_000)['count'],1)
    def test_tf(self):
        self.assertEqual(transform_point([1,2,3],[4,5,6],[0,0,0,1]),(5,7,9))
        self.assertEqual(transform_point([1,2,3],[0,0,0],[.5,-.5,.5,-.5]),(3,-1,-2))
        with self.assertRaises(ValueError): transform_point([0,0,0],[0,0,0],[0,0,0,0])

if __name__=='__main__': unittest.main()

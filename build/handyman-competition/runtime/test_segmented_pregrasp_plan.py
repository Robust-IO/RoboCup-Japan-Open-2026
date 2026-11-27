import unittest
from segmented_pregrasp_plan import build_plan


class PlanTests(unittest.TestCase):
    def test_no_motion_authority(self):
        p=build_plan();self.assertFalse(p['motion_authorized']);self.assertTrue(p['no_base_motion'])

    def test_single_joint_and_continuity(self):
        stages=build_plan()['stages']
        for i,s in enumerate(stages):
            self.assertEqual(sum(a!=b for a,b in zip(s['expected_before'],s['target'])),1)
            if i:self.assertEqual(stages[i-1]['target'],s['expected_before'])

    def test_wrist_before_flex(self):
        self.assertEqual([s['name'] for s in build_plan()['stages']],['lift','open','wrist_first','flex_second'])

    def test_no_descent_or_closure(self):
        for s in build_plan()['stages']:
            self.assertGreaterEqual(s['target'][0],s['expected_before'][0])
            self.assertGreaterEqual(s['target'][5],s['expected_before'][5])


if __name__=='__main__':unittest.main()

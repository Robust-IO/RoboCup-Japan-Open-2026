import unittest,math
from container_transfer import shorten_release_approach
class ReductionTests(unittest.TestCase):
 def test_seven_cm_along_heading(self):
  plan={'gripper_xy_map_m':[0,0],'opening':{'polygon_map_m':[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]}}
  result=shorten_release_approach(plan,-math.pi/2,.07)
  self.assertAlmostEqual(result['gripper_xy_map_m'][1],.07)
  self.assertEqual(plan['gripper_xy_map_m'],[0,0])
 def test_outside_opening_not_accepted(self):
  plan={'gripper_xy_map_m':[0,0],'opening':{'polygon_map_m':[[-.03,-.03],[.03,-.03],[.03,.03],[-.03,.03]]}}
  with self.assertRaises(ValueError):shorten_release_approach(plan,0,.07)

import unittest
from release_geometry import release_check
class ReleaseGeometryTests(unittest.TestCase):
    def check(self,xy=(0,0),z=.42):
        return release_check([*xy,.5],[[0,0,z]],{'plane_map':[0,0,.4],'destination_id':'t'},
          {'id':'t','axis_aligned_bounds':{'center':{'x':0,'y':0},'size':{'x':1,'y':1}}}, {'destination_id':'t'},(-.01,.04))
    def test_center_inside_at_height_allows_release(self):self.assertTrue(self.check()['ready'])
    def test_outside_or_too_high_does_not_release(self):
        self.assertFalse(self.check((.6,0))['ready']);self.assertFalse(self.check(z=.6)['ready'])

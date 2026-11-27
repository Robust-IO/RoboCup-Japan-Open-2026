import unittest
import numpy as np
from measure_destination_surface import measure


class TableDepthTest(unittest.TestCase):
    def fixture(self):
        depth=np.full((100,100),.5)
        snapshot=dict(rgb_k=[100,0,50,0,100,50,0,0,1],base_from_rgb=dict(translation=[0,0,1],quaternion=[1,0,0,0]),streams=dict(depth=dict(stamp_ns=1)),tracking_transforms=dict(map_from_base_at_depth=dict(stamp_ns=1,translation=[0,0,0],quaternion=[0,0,0,1])))
        dest=dict(id='table',room='bedroom',axis_aligned_bounds=dict(center=dict(x=0,y=0),size=dict(x=1,y=1)))
        return depth,snapshot,dest

    def test_horizontal_plane(self):
        result=measure(*self.fixture())
        self.assertAlmostEqual(result['table_height_map_m'],.5)
        self.assertFalse(result['motion_authorized'])

    def test_reject_wrong_time(self):
        depth,snapshot,dest=self.fixture();snapshot['tracking_transforms']['map_from_base_at_depth']['stamp_ns']=2
        with self.assertRaisesRegex(ValueError,'image_time'):measure(depth,snapshot,dest)

    def test_no_depth_not_a_table(self):
        depth,snapshot,dest=self.fixture();depth[:]=0
        with self.assertRaisesRegex(ValueError,'insufficient'):measure(depth,snapshot,dest)


if __name__=='__main__':unittest.main()

import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_visibility import visible_cells, region_interiors
from build_active_search_points import select_points


class VisibilityTests(unittest.TestCase):
    def test_wall_and_corner_block(self):
        grid=np.ones((9,9),bool);grid[4,4]=False
        self.assertFalse(visible_cells(grid,(2,4),[(6,4)])[0])
        grid[4,4]=True;grid[3,4]=False
        self.assertFalse(visible_cells(grid,(3,3),[(4,4)])[0])

    def test_partition_selects_other_side(self):
        grid=np.ones((80,80),bool);grid[12:68,39:42]=False
        room={'r':{'region':[[0,0],[8,0],[8,8],[0,8]]}}
        row=select_points(grid,.1,[0,0,0],(1,4),room,.15)[0]
        self.assertTrue(row['backups'])
        self.assertLess((row['main']['x']-4)*(row['backups'][0]['x']-4),0)
        self.assertGreater(row['backups'][0]['estimated_new_visible_area_m2'],.5)
        self.assertFalse(row['does_not_exist_authorized'])
        self.assertGreater(row['backups'][0]['interior_depth_grid_m'],.5)
        self.assertGreater(abs(row['backups'][0]['x']-4),1.)

    def test_region_center_not_opening(self):
        mask=np.zeros((15,25),bool);mask[2:13,2:13]=True
        mask[7,13:23]=True
        labels,depth,sizes=region_interiors(mask)
        self.assertEqual(len(sizes),1)
        self.assertGreater(depth[7,7],depth[7,16])
        self.assertEqual(depth.max(),depth[7,7])

    def test_regions_separate_and_empty(self):
        mask=np.zeros((12,12),bool);mask[1:4,1:4]=True;mask[7:11,7:11]=True
        labels,depth,sizes=region_interiors(mask)
        self.assertEqual(sorted(sizes.values()),[9,16])
        self.assertNotEqual(labels[2,2],labels[8,8])
        self.assertEqual(region_interiors(np.zeros((3,3),bool))[2],{})

    def test_open_room_does_not_add_redundant_backup(self):
        row=select_points(np.ones((30,30),bool),.1,[0,0,0],(.5,.5),
            {'r':{'region':[[0,0],[3,0],[3,3],[0,3]]}},.1)[0]
        self.assertEqual(row['backups'],[])

    def test_sealed_partition_never_selects_unreachable_side(self):
        grid=np.ones((40,40),bool);grid[:,20]=False
        row=select_points(grid,.1,[0,0,0],(.5,.5),
            {'r':{'region':[[0,0],[4,0],[4,4],[0,4]]}},.1)[0]
        self.assertTrue(all(p['x']<2 for p in [row['main']]+row['backups']))


if __name__=='__main__':unittest.main()

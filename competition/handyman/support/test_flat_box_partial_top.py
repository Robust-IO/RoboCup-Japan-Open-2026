import unittest,numpy as np,cv2
from unittest.mock import patch
from flat_box_partial_top import fit_points,locked_patch
class PartialTopTest(unittest.TestCase):
    model=dict(length_m=.084,width_m=.05,height_m=.022)
    def points(self,start=.020,stop=.042):
        x,y=np.meshgrid(np.linspace(start,stop,16),np.linspace(-.025,.025,20))
        return np.c_[x.ravel()+.7,y.ravel(),np.full(x.size,.511)]
    def test_occluded_near_edge_keeps_center_from_visible_far_edge(self):
        result=fit_points(self.points(),[.7,0,.5],[1,0,0],self.model)
        np.testing.assert_allclose(result['center_m'],[.7,0,.5],atol=.001)
        self.assertFalse(result['occluded_edges_measured'])
    def test_internal_patch_cannot_claim_outer_edge(self):
        with self.assertRaisesRegex(ValueError,'outer_box_edge'):
            fit_points(self.points(-.01,.01),[.7,0,.5],[1,0,0],self.model)
    def test_missing_depth_is_not_replaced_by_locked_pose(self):
        with self.assertRaises(ValueError):fit_points([], [.7,0,.5],[1,0,0],self.model)
    def patch_scene(self):
        p=self.points().reshape(20,16,3)
        hsv=np.empty((20,16,3),np.uint8);hsv[:]=[100,200,200]
        rgb=cv2.cvtColor(hsv,cv2.COLOR_HSV2RGB)
        return p,rgb,np.ones((20,16),float),dict(streams=dict(depth=dict(stamp_ns=123)))
    def test_locked_patch_marks_xy_as_prediction_and_forbids_further_approach(self):
        p,rgb,depth,snapshot=self.patch_scene()
        model=dict(self.model,grip_above_bottom_m=.035)
        with patch('flat_box_partial_top.pixel_points',return_value=p):
            result=locked_patch(rgb,depth,snapshot,[.7,0,.5],[1,0,0],model,dict(hue=100,tolerance=8))
        self.assertTrue(result['position_xy_is_prediction'])
        self.assertTrue(result['current_visible_surface_depth_measured'])
        self.assertFalse(result['additional_base_approach_authorized'])
        self.assertAlmostEqual(result['observed_top_z_m'],.511)
    def test_locked_patch_does_not_replace_missing_or_wrong_height_depth(self):
        p,rgb,depth,snapshot=self.patch_scene();model=dict(self.model,grip_above_bottom_m=.035)
        for missing in (True,False):
            if not missing:p=p.copy();p[:,:,2]+=.08
            with patch('flat_box_partial_top.pixel_points',return_value=p),self.assertRaises(ValueError):
                locked_patch(rgb,depth*0 if missing else depth,snapshot,[.7,0,.5],[1,0,0],model,dict(hue=100,tolerance=8))
if __name__=='__main__':unittest.main()

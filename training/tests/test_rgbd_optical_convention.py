"""Algebra checks for the audited HSR prefab; not runtime calibration proof."""
import unittest

class OpticalConventionTests(unittest.TestCase):
    def test_camera_to_published_parent_basis(self):
        # Image optical -> Unity camera: right, down, forward = (x,-y,z).
        # Camera child Rz(pi) -> parent: (-x,+y,z).
        # Published parent basis reflects Unity local X: (x,y,z).
        for optical in [(1,0,0),(0,1,0),(0,0,1),(.2,.3,1.7)]:
            x,y,z=optical
            camera=(x,-y,z)
            parent=(-camera[0],-camera[1],camera[2])
            ros=(-parent[0],parent[1],parent[2])
            self.assertEqual(ros,optical)
    def test_raster_flip(self):
        # ReadPixels rows start at the bottom; prior vertical shader flip
        # makes row zero refer to the visual top. Same for RGB and depth.
        height=480
        for row in [0,1,239,479]:
            sampled_from_bottom=height-1-row
            visual_row=height-1-sampled_from_bottom
            self.assertEqual(visual_row,row)

if __name__=='__main__': unittest.main()

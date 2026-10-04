import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from probe_search_path import path_valid


class PathProbeTests(unittest.TestCase):
    def test_valid(self):self.assertTrue(path_valid(4,'map',[[0,0],[1,1]],[1,1]))
    def test_wrong_frame(self):self.assertFalse(path_valid(4,'odom',[[1,1]],[1,1]))
    def test_far_endpoint(self):self.assertFalse(path_valid(4,'map',[[.5,.5]],[1,1]))
    def test_empty(self):self.assertFalse(path_valid(4,'map',[],[1,1]))
    def test_nonfinite(self):self.assertFalse(path_valid(4,'map',[[float('nan'),1]],[1,1]))
    def test_failed_action(self):self.assertFalse(path_valid(6,'map',[[1,1]],[1,1]))


if __name__=='__main__':unittest.main()

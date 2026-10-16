import importlib.util
from pathlib import Path
import unittest

s = importlib.util.spec_from_file_location('fusion', Path(__file__).resolve().parents[1]/'scripts/predict_rgb_readonly.py')
m = importlib.util.module_from_spec(s)
s.loader.exec_module(m)

class FusionTests(unittest.TestCase):
    def test_empty(self):
        self.assertEqual(m.classwise_keep([]), [])

    def test_same_class_duplicate_and_distinct_object(self):
        rows = [[0,0,10,10,.4,5], [0,0,10,10,.9,5], [20,0,30,10,.7,5]]
        self.assertEqual(m.classwise_keep(rows), [1,2])

    def test_cross_class_not_silently_suppressed(self):
        self.assertEqual(m.classwise_keep([[0,0,10,10,.8,1],[0,0,10,10,.7,5]]), [0,1])

    def test_degenerate(self):
        self.assertEqual(m.box_iou([0,0,0,0],[0,0,0,0]), 0)

if __name__ == '__main__':
    unittest.main()

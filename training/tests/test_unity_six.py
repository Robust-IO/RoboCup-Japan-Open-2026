import importlib.util
from pathlib import Path
import unittest
import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'

def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SixClassTests(unittest.TestCase):
    def test_disconnected_one_instance_with_hole(self):
        c = load('prepare_unity_six').converter
        ids = np.zeros((100, 100), dtype=np.uint8)
        ids[10:25, 40:60] = 6
        ids[30:90, 25:75] = 6
        ids[45:60, 40:55] = 0
        seg, boxes = c.annotations(ids, [dict(instanceId=6, classId=5, className='filled_ketchup', globalClassId=9)])
        self.assertEqual(len(seg), 1)
        self.assertEqual(len(boxes), 1)
        points = np.array(seg[0].split()[1:], dtype=np.float32).reshape(-1, 2) * 100
        self.assertGreaterEqual(c.training_iou((ids == 6).astype(np.uint8), points), .95)
        legacy = load('prepare_unity_pilot')
        with self.assertRaises(ValueError):
            legacy.hole_contour((ids == 6).astype(np.uint8))

    def test_ketchup_cyan_mask_and_mapping(self):
        c = load('prepare_unity_six').converter
        rgb = np.zeros((32, 32, 3), dtype=np.uint8)
        rgb[4:24, 8:20] = [0, 255, 255]
        ids = c.decode_ids(rgb)
        self.assertEqual(int(ids.max()), 6)
        instance = dict(instanceId=6, classId=5, className='filled_ketchup', globalClassId=9)
        segments, boxes = c.annotations(ids, [instance])
        self.assertTrue(segments[0].startswith('5 '))
        self.assertTrue(boxes[0].startswith('5 '))
        instance['globalClassId'] = 8
        with self.assertRaises(ValueError):
            c.annotations(ids, [instance])

    def test_legacy_entry_still_rejects_six(self):
        legacy = load('prepare_unity_pilot')
        load('prepare_unity_six')
        self.assertEqual(len(legacy.NAMES), 5)
        with self.assertRaises(ValueError):
            legacy.decode_ids(np.array([[[0, 255, 255]]], dtype=np.uint8))

    def test_inference_exact_registry(self):
        p = load('predict_rgb_readonly')
        names = ['apple', 'canned_juice', 'rabbit_doll', 'pink_cup', 'white_cup']
        self.assertEqual(p.validate_registry(dict(enumerate(names))), [0, 3, 15, 14, 26])
        self.assertEqual(p.validate_registry(names + ['filled_ketchup'])[-1], 9)
        with self.assertRaises(ValueError):
            p.validate_registry(names + ['empty_ketchup'])
        with self.assertRaises(ValueError):
            p.validate_registry(names + ['filled_ketchup', 'other'])

if __name__ == '__main__':
    unittest.main()

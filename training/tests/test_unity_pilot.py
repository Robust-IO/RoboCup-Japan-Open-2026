import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

spec = importlib.util.spec_from_file_location('pilot', Path(__file__).parents[1] / 'scripts' / 'prepare_unity_pilot.py')
pilot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pilot)


def instance(cls):
    return {'instanceId': cls+1, 'classId': cls, 'globalClassId': pilot.GLOBAL_IDS[cls], 'className': pilot.NAMES[cls]}


class PilotTests(unittest.TestCase):
    def test_rgb_bit_order(self):
        rgb = np.array([[[255, 0, 0], [0, 255, 0], [255, 255, 0], [0, 0, 255], [255, 0, 255]]], dtype=np.uint8)
        np.testing.assert_array_equal(pilot.decode_ids(rgb), [[1, 2, 3, 4, 5]])

    def test_bad_mask(self):
        for color in ([255, 255, 255], [50, 0, 0]):
            with self.assertRaises(ValueError):
                pilot.decode_ids(np.array([[color]], dtype=np.uint8))

    def test_bbox_top_left_origin_and_class_mapping(self):
        ids = np.zeros((100, 200), dtype=np.uint8)
        ids[10:30, 20:60] = 3
        seg, box = pilot.annotations(ids, [instance(2)])
        self.assertEqual(box, ['2 0.20000000 0.20000000 0.20000000 0.20000000'])
        self.assertTrue(seg[0].startswith('2 '))

    def test_hidden_instance(self):
        self.assertEqual(pilot.annotations(np.zeros((20, 20), np.uint8), [instance(0)]), ([], []))

    def test_missing_metadata(self):
        with self.assertRaises(ValueError):
            pilot.annotations(np.ones((20, 20), np.uint8), [])

    def test_holes_preserved_through_trainer(self):
        ids = np.zeros((100, 100), np.uint8)
        ids[5:95, 5:95] = 1
        ids[15:35, 15:35] = 0
        ids[50:80, 50:80] = 0
        segments, _ = pilot.annotations(ids, [instance(0)])
        points = np.array(segments[0].split()[1:], np.float32).reshape(-1, 2)*100
        self.assertGreaterEqual(pilot.contour_iou(ids, points), .95)
        self.assertGreaterEqual(pilot.training_iou(ids, points), .95)
        raster = np.zeros_like(ids)
        cv2.fillPoly(raster, [points.astype(np.int32)], 1)
        self.assertEqual(int(raster[25, 25]), 0)
        self.assertEqual(int(raster[65, 65]), 0)

    def test_islands_still_rejected(self):
        ids = np.zeros((100, 100), np.uint8)
        ids[5:45, 5:45] = 1
        ids[55:95, 55:95] = 1
        with self.assertRaises(ValueError):
            pilot.annotations(ids, [instance(0)])

    def test_trainer_distortion_is_not_ignored(self):
        with patch.object(pilot, 'training_iou', return_value=.94):
            with self.assertRaisesRegex(ValueError, 'trainer=0.940'):
                pilot.annotations(np.ones((20, 20), np.uint8), [instance(0)])

    def test_border_coordinates_remain_normalized(self):
        ids = np.ones((37, 53), np.uint8)
        seg, _ = pilot.annotations(ids, [instance(0)])
        values = np.array(seg[0].split()[1:], float)
        self.assertTrue(np.all((values >= 0) & (values <= 1)))

    def test_small_visible_instance_rejected(self):
        ids = np.zeros((100, 100), np.uint8)
        ids[0:2, 0:2] = 1
        with self.assertRaises(ValueError):
            pilot.annotations(ids, [instance(0)])

    def test_path_traversal(self):
        with self.assertRaises(ValueError):
            pilot.within(Path('/tmp/batch'), '../secret.png')

    def test_wrong_class_name(self):
        value = instance(0)
        value['className'] = 'white_cup'
        with self.assertRaises(ValueError):
            pilot.annotations(np.ones((20, 20), np.uint8), [value])

    def test_duplicate_instance(self):
        with self.assertRaises(ValueError):
            pilot.annotations(np.ones((20, 20), np.uint8), [instance(0), instance(0)])

    def make_batch(self, root, seed):
        for directory in ['frames', 'rgb', 'instance']:
            (root / directory).mkdir(parents=True)
        batch = {'schemaVersion': 1, 'seed': seed, 'count': 5, 'height': 64, 'width': 64,
                 'sources': [{'name': n, 'globalClassId': g} for n, g in zip(pilot.NAMES, pilot.GLOBAL_IDS)]}
        (root / 'batch.json').write_text(json.dumps(batch))
        for cls in range(5):
            stem = f'{cls:06d}'
            rgb = np.full((64, 64, 3), seed+cls, np.uint8)
            mask = np.zeros_like(rgb)
            iid = cls+1
            mask[10:30, 20:40] = [255*(iid & 1), 255*((iid >> 1) & 1), 255*((iid >> 2) & 1)]
            for directory, image in [('rgb', rgb), ('instance', mask)]:
                cv2.imwrite(str(root / directory / (stem+'.png')), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            frame = {'schemaVersion': 1, 'seed': seed, 'group': f'seed_{seed}', 'index': cls, 'width': 64, 'height': 64,
                     'rgb': f'rgb/{stem}.png', 'mask': f'instance/{stem}.png', 'negative': False, 'instances': [instance(cls)]}
            (root / 'frames' / (stem+'.json')).write_text(json.dumps(frame))

    def test_end_to_end_and_refuse_overwrite(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            self.make_batch(root/'a', 10)
            self.make_batch(root/'b', 20)
            splits = {'train': [root/'a'], 'val': [root/'b'], 'test': []}
            report = pilot.prepare(splits, root/'out')
            self.assertEqual(report['accepted'], {'train': 5, 'val': 5})
            self.assertEqual(report['rejected'], [])
            self.assertEqual(report['classCounts']['train'], [1]*5)
            self.assertTrue((root/'out/dataset-seg.yaml').exists())
            self.assertTrue((root/'out/detect/labels/train/s10_000000.txt').exists())
            with self.assertRaises(ValueError):
                pilot.prepare(splits, root/'out')

    def test_duplicate_seed_refused_before_writes(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            self.make_batch(root/'a', 10)
            self.make_batch(root/'b', 10)
            with self.assertRaises(ValueError):
                pilot.prepare({'train': [root/'a'], 'val': [root/'b']}, root/'out')
            self.assertFalse((root/'out').exists())

    def test_inspect_single_batch_without_training_yaml(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            self.make_batch(root/'a', 10)
            report = pilot.prepare({'inspect': [root/'a']}, root/'out', inspect_only=True)
            self.assertEqual(report['accepted'], {'inspect': 5})
            self.assertTrue((root/'out/previews/inspect/s10_000000.png').exists())
            self.assertFalse((root/'out/dataset-seg.yaml').exists())

    def test_missing_mask_is_quarantined(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            self.make_batch(root/'a', 10)
            (root/'a/instance/000000.png').unlink()
            report = pilot.prepare({'inspect': [root/'a']}, root/'out', inspect_only=True)
            self.assertEqual(report['accepted'], {'inspect': 4})
            self.assertEqual(len(report['rejected']), 1)


if __name__ == '__main__':
    unittest.main()

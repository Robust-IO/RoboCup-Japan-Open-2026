import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


TRAINING_ROOT = Path(__file__).resolve().parents[1]


def load_module(module_name, relative_path, injected_modules=None):
    module_path = TRAINING_ROOT / relative_path
    if not module_path.is_file():
        raise FileNotFoundError(module_path)

    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    injected_modules = injected_modules or {}
    with mock.patch.dict(sys.modules, injected_modules, clear=False):
        spec.loader.exec_module(module)
    return module


class SplitDatasetTests(unittest.TestCase):
    def test_split_dataset_uses_flat_raw_export_layout(self):
        split_module = load_module(
            'split_dataset',
            Path('scripts/split_dataset.py'),
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            raw_dir = root / 'raw'
            output_root = root / 'output'
            canonical_classes = root / 'classes.txt'

            (raw_dir / 'images').mkdir(parents=True)
            (raw_dir / 'labels').mkdir(parents=True)
            canonical_classes.write_text('apple\nspray_bottle\nsugar\n', encoding='utf-8')
            (raw_dir / 'classes.txt').write_text(
                canonical_classes.read_text(encoding='utf-8'),
                encoding='utf-8',
            )
            (raw_dir / 'notes.json').write_text(
                json.dumps({'categories': [{'id': '0', 'name': 'apple'}]}),
                encoding='utf-8',
            )

            for name in ('a.jpg', 'b.jpg', 'c.png', 'd.jpg'):
                (raw_dir / 'images' / name).write_bytes(b'image')

            (raw_dir / 'labels' / 'a.txt').write_text('0 0.5 0.5 0.2 0.2\n', encoding='utf-8')
            (raw_dir / 'labels' / 'c.txt').write_text('2 0.4 0.4 0.1 0.1\n', encoding='utf-8')

            split_module.main([
                '--raw-dir', str(raw_dir),
                '--output-root', str(output_root),
                '--classes-file', str(canonical_classes),
                '--train-ratio', '0.5',
                '--val-ratio', '0.25',
                '--test-ratio', '0.25',
                '--seed', '7',
            ])

            split_images = list(output_root.glob('images/*/*'))
            split_labels = list(output_root.glob('labels/*/*'))
            self.assertEqual(4, len(split_images))
            self.assertEqual(4, len(split_labels))
            self.assertEqual(2, len(list((output_root / 'images' / 'train').iterdir())))
            self.assertEqual(1, len(list((output_root / 'images' / 'val').iterdir())))
            self.assertEqual(1, len(list((output_root / 'images' / 'test').iterdir())))

            all_label_names = {path.name for path in split_labels}
            self.assertEqual({'a.txt', 'b.txt', 'c.txt', 'd.txt'}, all_label_names)
            copied_empty = [
                path for path in split_labels
                if path.name in {'b.txt', 'd.txt'} and path.read_text(encoding='utf-8') == ''
            ]
            self.assertEqual(2, len(copied_empty))

    def test_split_dataset_rejects_mismatched_class_lists(self):
        split_module = load_module(
            'split_dataset_mismatch',
            Path('scripts/split_dataset.py'),
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            raw_dir = root / 'raw'
            output_root = root / 'output'
            canonical_classes = root / 'classes.txt'

            (raw_dir / 'images').mkdir(parents=True)
            (raw_dir / 'labels').mkdir(parents=True)
            (raw_dir / 'images' / 'a.jpg').write_bytes(b'image')
            (raw_dir / 'classes.txt').write_text('Sugar\n', encoding='utf-8')
            canonical_classes.write_text('sugar\n', encoding='utf-8')

            with self.assertRaises(SystemExit) as ctx:
                split_module.main([
                    '--raw-dir', str(raw_dir),
                    '--output-root', str(output_root),
                    '--classes-file', str(canonical_classes),
                ])

            self.assertIn('Class list mismatch', str(ctx.exception))


class TrainScriptTests(unittest.TestCase):
    def test_train_script_accepts_default_yolo26_alias(self):
        class FakeYOLO:
            last_model = None
            last_kwargs = None

            def __init__(self, model_spec):
                FakeYOLO.last_model = model_spec

            def train(self, **kwargs):
                FakeYOLO.last_kwargs = kwargs
                return {'ok': True}

        fake_ultralytics = types.SimpleNamespace(YOLO=FakeYOLO)
        train_module = load_module(
            'train_script',
            Path('scripts/train.py'),
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            data_path = root / 'dataset.yaml'
            project_dir = root / 'runs'
            data_path.write_text('path: dataset\ntrain: images/train\nval: images/val\n', encoding='utf-8')

            with mock.patch.dict(sys.modules, {'ultralytics': fake_ultralytics}, clear=False):
                train_module.main([
                    '--data', str(data_path),
                    '--project', str(project_dir),
                    '--name', 'demo',
                ])

            self.assertEqual('yolo26n.pt', Path(FakeYOLO.last_model).name)
            self.assertEqual(str(data_path.resolve()), FakeYOLO.last_kwargs['data'])
            self.assertEqual(str(project_dir.resolve()), FakeYOLO.last_kwargs['project'])
            self.assertEqual('demo', FakeYOLO.last_kwargs['name'])


class ExportXAnyLabelingTests(unittest.TestCase):
    def test_export_script_generates_xanylabeling_model_dir_from_run_name(self):
        class FakeYOLO:
            last_model = None
            last_export_kwargs = None

            def __init__(self, model_spec):
                FakeYOLO.last_model = model_spec

            def export(self, **kwargs):
                FakeYOLO.last_export_kwargs = kwargs
                export_path = Path(kwargs['project']) / kwargs['name'] / 'weights.onnx'
                export_path.parent.mkdir(parents=True, exist_ok=True)
                export_path.write_bytes(b'onnx')
                return str(export_path)

        fake_ultralytics = types.SimpleNamespace(YOLO=FakeYOLO)

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            runs_root = root / 'runs'
            weights_path = runs_root / 'simple-20' / 'weights' / 'best.pt'
            classes_path = root / 'classes.txt'
            output_dir = root / 'exports' / 'simple-20'

            weights_path.parent.mkdir(parents=True)
            weights_path.write_bytes(b'pt')
            classes_path.write_text('apple\nspray_bottle\nsugar\n', encoding='utf-8')

            export_module = load_module(
                'export_xanylabeling',
                Path('scripts/export_xanylabeling.py'),
                injected_modules={'ultralytics': fake_ultralytics},
            )

            with mock.patch.dict(sys.modules, {'ultralytics': fake_ultralytics}, clear=False):
                export_module.main([
                    '--run-name', 'simple-20',
                    '--runs-root', str(runs_root),
                    '--classes-file', str(classes_path),
                    '--output-dir', str(output_dir),
                    '--display-name', 'simple-20 auto labeler',
                    '--conf-threshold', '0.35',
                    '--iou-threshold', '0.60',
                    '--imgsz', '640',
                ])

            config_path = output_dir / 'config.yaml'
            model_path = output_dir / 'model.onnx'

            self.assertEqual(str(weights_path.resolve()), FakeYOLO.last_model)
            self.assertEqual('onnx', FakeYOLO.last_export_kwargs['format'])
            self.assertEqual(640, FakeYOLO.last_export_kwargs['imgsz'])
            self.assertTrue(model_path.is_file())
            self.assertTrue(config_path.is_file())

            config_text = config_path.read_text(encoding='utf-8')
            self.assertIn('type: yolo26\n', config_text)
            self.assertIn('name: simple-20-xanylabeling\n', config_text)
            self.assertIn('display_name: simple-20 auto labeler\n', config_text)
            self.assertIn('model_path: model.onnx\n', config_text)
            self.assertIn('iou_threshold: 0.6\n', config_text)
            self.assertIn('conf_threshold: 0.35\n', config_text)
            self.assertIn('max_det: 300\n', config_text)
            self.assertIn('  - apple\n', config_text)
            self.assertIn('  - spray_bottle\n', config_text)
            self.assertIn('  - sugar\n', config_text)

    def test_export_script_accepts_direct_weights_path(self):
        class FakeYOLO:
            last_model = None

            def __init__(self, model_spec):
                FakeYOLO.last_model = model_spec

            def export(self, **kwargs):
                export_path = Path(kwargs['project']) / kwargs['name'] / 'raw.onnx'
                export_path.parent.mkdir(parents=True, exist_ok=True)
                export_path.write_bytes(b'onnx')
                return str(export_path)

        fake_ultralytics = types.SimpleNamespace(YOLO=FakeYOLO)

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            weights_path = root / 'runs' / 'simple-20' / 'weights' / 'best.pt'
            classes_path = root / 'classes.txt'
            output_dir = root / 'model_dir'

            weights_path.parent.mkdir(parents=True)
            weights_path.write_bytes(b'pt')
            classes_path.write_text('apple\n', encoding='utf-8')

            export_module = load_module(
                'export_xanylabeling_direct',
                Path('scripts/export_xanylabeling.py'),
                injected_modules={'ultralytics': fake_ultralytics},
            )

            with mock.patch.dict(sys.modules, {'ultralytics': fake_ultralytics}, clear=False):
                export_module.main([
                    '--weights', str(weights_path),
                    '--classes-file', str(classes_path),
                    '--output-dir', str(output_dir),
                ])

            self.assertEqual(str(weights_path.resolve()), FakeYOLO.last_model)
            self.assertTrue((output_dir / 'config.yaml').is_file())
            self.assertTrue((output_dir / 'model.onnx').is_file())
            self.assertIn(
                'name: simple-20-xanylabeling\n',
                (output_dir / 'config.yaml').read_text(encoding='utf-8'),
            )


class ExtractFramesTests(unittest.TestCase):
    def test_extract_frames_parser_uses_flat_raw_output_root(self):
        fake_cv2 = types.SimpleNamespace()
        extract_module = load_module(
            'extract_frames',
            Path('scripts/extract_frames.py'),
            injected_modules={'cv2': fake_cv2},
        )

        args = extract_module.parse_args(['--video', 'clip.mp4'])
        self.assertEqual(['clip.mp4'], args.video)
        self.assertEqual('training/data/raw', args.output_root)
        self.assertFalse(hasattr(args, 'session'))


if __name__ == '__main__':
    unittest.main()

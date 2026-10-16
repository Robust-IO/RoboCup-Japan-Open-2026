#!/usr/bin/env python3

import argparse
import re
import shutil
import tempfile
from pathlib import Path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Export a trained YOLO checkpoint for X-AnyLabeling.',
    )
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument(
        '--weights',
        help='Direct path to a trained .pt checkpoint.',
    )
    source_group.add_argument(
        '--run-name',
        help='Training run name under training/runs/<name>/weights/best.pt.',
    )
    parser.add_argument(
        '--runs-root',
        default='training/runs',
        help='Training runs root used with --run-name.',
    )
    parser.add_argument(
        '--classes-file',
        default='training/classes.txt',
        help='Classes file used to populate the X-AnyLabeling config.',
    )
    parser.add_argument(
        '--output-dir',
        default=None,
        help='Directory where model.onnx and config.yaml will be written.',
    )
    parser.add_argument(
        '--display-name',
        default=None,
        help='Display name shown by X-AnyLabeling.',
    )
    parser.add_argument(
        '--provider',
        default='Ultralytics',
        help='Provider field written to config.yaml.',
    )
    parser.add_argument(
        '--model-type',
        default='yolo26',
        help='X-AnyLabeling model type. Defaults to yolo26.',
    )
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--conf-threshold', type=float, default=0.25)
    parser.add_argument('--iou-threshold', type=float, default=0.70)
    parser.add_argument('--max-det', type=int, default=300)
    parser.add_argument('--opset', type=int, default=13)
    parser.add_argument(
        '--no-simplify',
        action='store_true',
        help='Disable ONNX graph simplification during export.',
    )
    return parser.parse_args(argv)


def load_yolo_class():
    try:
        from ultralytics import YOLO
    except ModuleNotFoundError as exc:
        raise SystemExit(
            'ultralytics is not installed. Run this command inside the pixi '
            'environment or install the dependency first.'
        ) from exc
    return YOLO


def sanitize_name(value: str) -> str:
    cleaned = re.sub(r'[^A-Za-z0-9._-]+', '-', value.strip())
    cleaned = cleaned.strip('-._')
    return cleaned or 'model'


def resolve_weights_path(args):
    if args.weights:
        weights_path = Path(args.weights).expanduser().resolve()
    else:
        weights_path = (
            Path(args.runs_root).expanduser().resolve()
            / args.run_name
            / 'weights'
            / 'best.pt'
        )

    if not weights_path.is_file():
        raise SystemExit(f'Checkpoint not found: {weights_path}')
    return weights_path


def infer_source_name(args, weights_path: Path) -> str:
    if args.run_name:
        return sanitize_name(args.run_name)
    if (
        weights_path.parent.name == 'weights'
        and weights_path.stem in {'best', 'last'}
        and weights_path.parent.parent.name
    ):
        return sanitize_name(weights_path.parent.parent.name)
    return sanitize_name(weights_path.stem)


def resolve_output_dir(args, source_name: str) -> Path:
    if args.output_dir:
        return Path(args.output_dir).expanduser().resolve()
    return (
        Path('training/exports').resolve()
        / 'xanylabeling'
        / source_name
    )


def load_classes(classes_file: Path):
    if not classes_file.is_file():
        raise SystemExit(f'Classes file not found: {classes_file}')
    classes = [
        line.strip()
        for line in classes_file.read_text(encoding='utf-8').splitlines()
        if line.strip()
    ]
    if not classes:
        raise SystemExit(f'Classes file is empty: {classes_file}')
    return classes


def _find_exported_onnx(export_result, export_root: Path) -> Path:
    if export_result:
        candidate = Path(str(export_result)).expanduser()
        if candidate.is_file():
            return candidate.resolve()

    matches = sorted(export_root.rglob('*.onnx'))
    if len(matches) == 1:
        return matches[0].resolve()
    if not matches:
        raise SystemExit('Ultralytics export did not produce an ONNX file.')
    raise SystemExit(
        f'Ultralytics export produced multiple ONNX files under {export_root}; '
        'unable to determine which one to use.'
    )


def export_onnx(weights_path: Path, output_dir: Path, args):
    output_dir.mkdir(parents=True, exist_ok=True)
    yolo_class = load_yolo_class()
    exporter = yolo_class(str(weights_path))

    with tempfile.TemporaryDirectory(prefix='xanylabeling-export-') as tmpdir:
        export_root = Path(tmpdir)
        export_result = exporter.export(
            format='onnx',
            imgsz=args.imgsz,
            opset=args.opset,
            simplify=not args.no_simplify,
            dynamic=False,
            project=str(export_root),
            name='model',
            exist_ok=True,
        )
        exported_onnx = _find_exported_onnx(export_result, export_root)
        final_model_path = output_dir / 'model.onnx'
        shutil.move(str(exported_onnx), str(final_model_path))

    return final_model_path


def _format_scalar(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        return str(value)
    return str(value)


def build_config_text(args, source_name: str, classes):
    display_name = args.display_name or f'{source_name} auto labeler'
    config_name = f'{source_name}-xanylabeling'
    lines = [
        f'type: {args.model_type}',
        f'name: {config_name}',
        f'provider: {args.provider}',
        f'display_name: {display_name}',
        'model_path: model.onnx',
        f'iou_threshold: {_format_scalar(args.iou_threshold)}',
        f'conf_threshold: {_format_scalar(args.conf_threshold)}',
        f'max_det: {args.max_det}',
        'classes:',
    ]
    lines.extend(f'  - {class_name}' for class_name in classes)
    return '\n'.join(lines) + '\n'


def write_config(output_dir: Path, config_text: str):
    config_path = output_dir / 'config.yaml'
    config_path.write_text(config_text, encoding='utf-8')
    return config_path


def main(argv=None):
    args = parse_args(argv)
    weights_path = resolve_weights_path(args)
    source_name = infer_source_name(args, weights_path)
    output_dir = resolve_output_dir(args, source_name)
    classes_file = Path(args.classes_file).expanduser().resolve()
    classes = load_classes(classes_file)

    output_dir.mkdir(parents=True, exist_ok=True)
    export_onnx(weights_path, output_dir, args)
    config_text = build_config_text(args, source_name, classes)
    write_config(output_dir, config_text)

    print(f'Wrote X-AnyLabeling model directory to: {output_dir}')


if __name__ == '__main__':
    main()

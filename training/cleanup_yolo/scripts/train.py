#!/usr/bin/env python3

import argparse
from pathlib import Path

from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(
        description='Train a custom YOLO detector for Interactive Cleanup.'
    )
    parser.add_argument(
        '--data',
        default='training/cleanup_yolo/dataset.yaml',
        help='Dataset YAML file.',
    )
    parser.add_argument(
        '--model',
        default='src/cleanup_vision_ros2/models/cleanup_model.pt',
        help='Base model checkpoint.',
    )
    parser.add_argument('--epochs', type=int, default=120)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument(
        '--project',
        default='training/cleanup_yolo/runs',
        help='Ultralytics project output directory.',
    )
    parser.add_argument('--name', default='cleanup_baseline')
    parser.add_argument(
        '--device',
        default=None,
        help='Optional device override, such as cpu or 0.',
    )
    parser.add_argument('--degrees', type=float, default=8.0)
    parser.add_argument('--translate', type=float, default=0.05)
    parser.add_argument('--scale', type=float, default=0.25)
    parser.add_argument('--hsv-h', type=float, default=0.015)
    parser.add_argument('--hsv-s', type=float, default=0.5)
    parser.add_argument('--hsv-v', type=float, default=0.3)
    parser.add_argument('--fliplr', type=float, default=0.5)
    parser.add_argument('--mosaic', type=float, default=1.0)
    parser.add_argument('--close-mosaic', type=int, default=10)
    parser.add_argument('--mixup', type=float, default=0.0)
    parser.add_argument('--copy-paste', type=float, default=0.0)
    parser.add_argument(
        '--cache',
        action='store_true',
        help='Cache images in memory/disk if supported.',
    )
    return parser.parse_args()


def main():
    args = parse_args()

    data_path = Path(args.data).resolve()
    model_path = Path(args.model).resolve()
    project_dir = Path(args.project).resolve()
    project_dir.mkdir(parents=True, exist_ok=True)

    if not data_path.is_file():
        raise SystemExit(f'Dataset YAML not found: {data_path}')
    if not model_path.is_file():
        raise SystemExit(f'Base model not found: {model_path}')

    trainer = YOLO(str(model_path))
    kwargs = dict(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        project=str(project_dir),
        name=args.name,
        degrees=args.degrees,
        translate=args.translate,
        scale=args.scale,
        hsv_h=args.hsv_h,
        hsv_s=args.hsv_s,
        hsv_v=args.hsv_v,
        fliplr=args.fliplr,
        mosaic=args.mosaic,
        close_mosaic=args.close_mosaic,
        mixup=args.mixup,
        copy_paste=args.copy_paste,
        cache=args.cache,
    )
    if args.device:
        kwargs['device'] = args.device

    trainer.train(**kwargs)


if __name__ == '__main__':
    main()

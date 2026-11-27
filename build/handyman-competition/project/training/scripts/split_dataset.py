#!/usr/bin/env python3

import argparse
import random
import shutil
from pathlib import Path


IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}
SPLITS = ('train', 'val', 'test')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Split a flat Label Studio YOLO export into train/val/test.',
    )
    parser.add_argument(
        '--raw-dir',
        default='training/data/raw',
        help='Flat raw export directory containing images/, labels/, classes.txt.',
    )
    parser.add_argument(
        '--output-root',
        default='training/data',
        help='Dataset root containing images/ and labels/ split folders.',
    )
    parser.add_argument(
        '--classes-file',
        default='training/classes.txt',
        help='Canonical class list that raw/classes.txt must match.',
    )
    parser.add_argument('--train-ratio', type=float, default=0.7)
    parser.add_argument('--val-ratio', type=float, default=0.2)
    parser.add_argument('--test-ratio', type=float, default=0.1)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument(
        '--clean',
        action='store_true',
        help='Delete existing generated files in split directories first.',
    )
    return parser.parse_args(argv)


def load_class_names(path: Path):
    if not path.is_file():
        raise SystemExit(f'Class file not found: {path}')

    names = [
        line.strip() for line in path.read_text(encoding='utf-8').splitlines()
        if line.strip()
    ]
    if not names:
        raise SystemExit(f'Class file is empty: {path}')
    return names


def validate_class_list(raw_classes_path: Path, canonical_classes_path: Path):
    raw_names = load_class_names(raw_classes_path)
    canonical_names = load_class_names(canonical_classes_path)
    if raw_names == canonical_names:
        return

    message = [
        'Class list mismatch between raw export and canonical classes file.',
        f'Raw: {raw_classes_path}',
        f'Canonical: {canonical_classes_path}',
    ]

    if set(raw_names) == set(canonical_names) and len(raw_names) == len(canonical_names):
        message.append('The class names match as a set, but the order differs.')
    else:
        message.append(f'Raw classes: {raw_names}')
        message.append(f'Canonical classes: {canonical_names}')

    raise SystemExit('\n'.join(message))


def list_raw_images(raw_dir: Path):
    images_dir = raw_dir / 'images'
    if not images_dir.is_dir():
        raise SystemExit(f'Raw images directory does not exist: {images_dir}')

    image_paths = sorted(
        path for path in images_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )
    if not image_paths:
        raise SystemExit(f'No images found under: {images_dir}')
    return image_paths


def split_items(items, train_ratio, val_ratio, test_ratio, seed):
    total = train_ratio + val_ratio + test_ratio
    if total <= 0:
        raise SystemExit('Split ratios must sum to a positive value.')

    normalized = [
        train_ratio / total,
        val_ratio / total,
        test_ratio / total,
    ]

    items = list(items)
    random.Random(seed).shuffle(items)

    n = len(items)
    n_train = int(round(n * normalized[0]))
    n_val = int(round(n * normalized[1]))
    n_train = min(n_train, n)
    n_val = min(n_val, n - n_train)
    n_test = n - n_train - n_val

    return {
        'train': items[:n_train],
        'val': items[n_train:n_train + n_val],
        'test': items[n_train + n_val:n_train + n_val + n_test],
    }


def clear_split_dir(path: Path):
    for child in path.iterdir():
        if child.name == '.gitkeep':
            continue
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()


def prepare_output_dirs(output_root: Path, clean: bool):
    for group in ('images', 'labels'):
        for split in SPLITS:
            split_dir = output_root / group / split
            split_dir.mkdir(parents=True, exist_ok=True)
            existing = [p for p in split_dir.iterdir() if p.name != '.gitkeep']
            if existing and not clean:
                raise SystemExit(
                    f'Split directory is not empty: {split_dir}\n'
                    'Re-run with --clean to regenerate the split.'
                )
            if clean:
                clear_split_dir(split_dir)


def copy_image_and_label(image_path: Path, raw_labels_dir: Path, output_root: Path, split: str):
    stem = image_path.stem
    dst_image = output_root / 'images' / split / image_path.name
    shutil.copy2(image_path, dst_image)

    src_label = raw_labels_dir / f'{stem}.txt'
    dst_label = output_root / 'labels' / split / f'{stem}.txt'
    if src_label.is_file():
        shutil.copy2(src_label, dst_label)
    else:
        dst_label.write_text('', encoding='utf-8')


def main(argv=None):
    args = parse_args(argv)

    raw_dir = Path(args.raw_dir)
    output_root = Path(args.output_root)
    canonical_classes_path = Path(args.classes_file)
    raw_labels_dir = raw_dir / 'labels'
    raw_classes_path = raw_dir / 'classes.txt'

    if not raw_dir.is_dir():
        raise SystemExit(f'Raw directory does not exist: {raw_dir}')
    if not raw_labels_dir.is_dir():
        raise SystemExit(f'Raw labels directory does not exist: {raw_labels_dir}')

    validate_class_list(raw_classes_path, canonical_classes_path)
    image_paths = list_raw_images(raw_dir)
    split_map = split_items(
        items=image_paths,
        train_ratio=args.train_ratio,
        val_ratio=args.val_ratio,
        test_ratio=args.test_ratio,
        seed=args.seed,
    )
    prepare_output_dirs(output_root, clean=args.clean)

    print('Image split:')
    for split in SPLITS:
        print(f'  {split}: {len(split_map[split])} image(s)')
        for image_path in split_map[split]:
            copy_image_and_label(image_path, raw_labels_dir, output_root, split)


if __name__ == '__main__':
    main()

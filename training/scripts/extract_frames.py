#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Extract frames from capture videos into training/data/raw/images.',
    )
    parser.add_argument(
        '--video',
        action='append',
        required=True,
        help='Input video path. Repeat this flag to add multiple videos.',
    )
    parser.add_argument(
        '--output-root',
        default='training/data/raw',
        help='Raw dataset root containing images/ and labels/.',
    )
    parser.add_argument(
        '--prefix',
        default=None,
        help='Optional filename prefix for generated frames.',
    )
    parser.add_argument(
        '--every-seconds',
        type=float,
        default=0.4,
        help='Sample one frame every N seconds.',
    )
    parser.add_argument(
        '--jpeg-quality',
        type=int,
        default=95,
        help='JPEG quality from 0 to 100.',
    )
    parser.add_argument(
        '--max-frames',
        type=int,
        default=0,
        help='Optional cap on total saved frames. 0 means unlimited.',
    )
    parser.add_argument(
        '--force',
        action='store_true',
        help='Allow writing into a non-empty raw images directory.',
    )
    return parser.parse_args(argv)


def ensure_output_dir(path: Path, force: bool):
    path.mkdir(parents=True, exist_ok=True)
    if force:
        return

    has_files = any(child.is_file() for child in path.iterdir())
    if has_files:
        raise SystemExit(
            f'Output directory is not empty: {path}\n'
            'Use --force or clean the directory first.'
        )


def extract_video(
    video_path: Path,
    output_dir: Path,
    every_seconds: float,
    jpeg_quality: int,
    max_frames: int,
    start_index: int,
    prefix: str,
):
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f'Failed to open video: {video_path}')

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps is None or fps <= 0:
        fps = 30.0
    frame_interval = max(1, int(round(fps * every_seconds)))

    saved = 0
    frame_idx = 0
    next_index = start_index

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        if frame_idx % frame_interval == 0:
            out_path = output_dir / f'{prefix}_frame_{next_index:06d}.jpg'
            cv2.imwrite(
                str(out_path),
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
            )
            saved += 1
            next_index += 1
            if max_frames > 0 and saved >= max_frames:
                break

        frame_idx += 1

    cap.release()
    return saved, next_index


def main(argv=None):
    args = parse_args(argv)

    raw_dir = Path(args.output_root)
    images_dir = raw_dir / 'images'
    labels_dir = raw_dir / 'labels'
    ensure_output_dir(images_dir, args.force)
    labels_dir.mkdir(parents=True, exist_ok=True)

    next_index = 1
    total_saved = 0
    for video in args.video:
        if args.max_frames > 0 and total_saved >= args.max_frames:
            break

        remaining = 0
        if args.max_frames > 0:
            remaining = args.max_frames - total_saved

        video_path = Path(video)
        prefix = args.prefix or video_path.stem
        saved, next_index = extract_video(
            video_path=video_path,
            output_dir=images_dir,
            every_seconds=args.every_seconds,
            jpeg_quality=args.jpeg_quality,
            max_frames=remaining,
            start_index=next_index,
            prefix=prefix,
        )
        total_saved += saved

    print(f'Raw dir: {raw_dir}')
    print(f'Saved frames: {total_saved}')
    print(f'Images dir: {images_dir}')
    print(f'Labels dir: {labels_dir}')


if __name__ == '__main__':
    main()

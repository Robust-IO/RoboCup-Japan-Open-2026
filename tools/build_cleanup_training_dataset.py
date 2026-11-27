#!/usr/bin/env python3
"""Build the 13-class Interactive Cleanup YOLO dataset.

The builder combines legacy Unity YOLO exports with X-AnyLabeling JSON files.
It fixes the vertically inverted v2 Unity export, remaps the old 30-class IDs by
name, clips slightly out-of-frame manual boxes, removes exact duplicate manual
images, and keeps adjacent manual captures together during dataset splitting.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import random
import shutil

from PIL import Image, ImageOps


CLASSES = [
    "bear_doll", "canned_juice", "dog_doll", "filled_ketchup",
    "filled_plastic_bottle", "hourglass", "rabbit_doll", "sauce",
    "soysauce", "spray_bottle", "sugar", "toy_penguin", "tumbler",
]
CLASS_IDS = {name: index for index, name in enumerate(CLASSES)}
SPLITS = ("train", "val", "test")
MANUAL_SESSIONS = {
    "20261013_canned_juice_01": "canned_juice",
    "20261013_filled_ketchup_01": "filled_ketchup",
    "20261013_sauce_01": "sauce",
    "20261013_spray_bottle_01": "spray_bottle",
    "20261013_sugar_01": "sugar",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--auto-source", action="append", type=Path, required=True,
        help="Unity YOLO export directory; specify once per v2/v3/v4 source.",
    )
    parser.add_argument(
        "--manual-root", type=Path,
        default=Path("training/cleanup_yolo/data/raw"),
    )
    parser.add_argument(
        "--output", type=Path, default=Path("training/cleanup_yolo/data"),
    )
    parser.add_argument("--manual-chunk-size", type=int, default=12)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--flip-source-name", action="append",
        default=["SyntheticYoloDataset_v2"],
        help="Unity source basename whose images and labels require vertical flipping.",
    )
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def clean_output(output: Path) -> None:
    for group in ("images", "labels"):
        for split in SPLITS:
            directory = output / group / split
            directory.mkdir(parents=True, exist_ok=True)
            existing = [path for path in directory.iterdir() if path.name != ".gitkeep"]
            if existing:
                for path in existing:
                    if path.is_dir():
                        shutil.rmtree(path)
                    else:
                        path.unlink()


def ensure_empty_output(output: Path, clean: bool) -> None:
    occupied = []
    for group in ("images", "labels"):
        for split in SPLITS:
            directory = output / group / split
            if directory.is_dir():
                occupied.extend(path for path in directory.iterdir() if path.name != ".gitkeep")
    if occupied and not clean:
        raise SystemExit(f"Output contains generated files; use --clean: {output}")
    if clean:
        clean_output(output)
    else:
        for group in ("images", "labels"):
            for split in SPLITS:
                (output / group / split).mkdir(parents=True, exist_ok=True)


def write_metadata(output: Path) -> None:
    (output / "classes.txt").write_text("\n".join(CLASSES) + "\n", encoding="utf-8")
    lines = [
        f"path: {output.resolve()}",
        "train: images/train",
        "val: images/val",
        "test: images/test",
        "",
        "names:",
    ]
    lines.extend(f"  {index}: {name}" for index, name in enumerate(CLASSES))
    (output / "dataset.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")


def image_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def copy_sample(
    image: Image.Image,
    labels: list[tuple[int, float, float, float, float]],
    output: Path,
    split: str,
    basename: str,
) -> None:
    image_path = output / "images" / split / f"{basename}.png"
    label_path = output / "labels" / split / f"{basename}.txt"
    image.convert("RGB").save(image_path, "PNG")
    lines = [
        f"{class_id} {x:.6f} {y:.6f} {width:.6f} {height:.6f}"
        for class_id, x, y, width, height in labels
    ]
    label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def parse_unity_labels(
    label_path: Path,
    source_classes: list[str],
    flip_vertical: bool,
) -> list[tuple[int, float, float, float, float]]:
    labels = []
    for line_number, line in enumerate(label_path.read_text().splitlines(), start=1):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{label_path}:{line_number}: expected 5 fields")
        old_id = int(fields[0])
        if not 0 <= old_id < len(source_classes):
            raise ValueError(f"{label_path}:{line_number}: invalid class ID {old_id}")
        name = source_classes[old_id]
        if name not in CLASS_IDS:
            continue
        x, y, width, height = map(float, fields[1:])
        if flip_vertical:
            y = 1.0 - y
        if not (
            0.0 <= x <= 1.0 and 0.0 <= y <= 1.0
            and 0.0 < width <= 1.0 and 0.0 < height <= 1.0
        ):
            raise ValueError(f"{label_path}:{line_number}: invalid normalized box")
        labels.append((CLASS_IDS[name], x, y, width, height))
    return labels


def add_unity_sources(
    sources: list[Path], output: Path, flip_names: set[str], report: dict,
) -> None:
    for source in sources:
        source = source.resolve()
        class_file = source / "classes.txt"
        if not class_file.is_file():
            raise ValueError(f"Missing {class_file}")
        source_classes = [
            line.strip()
            for line in class_file.read_text(encoding="utf-8-sig").splitlines()
            if line.strip()
        ]
        flip_vertical = source.name in flip_names
        source_report = {"flip_vertical": flip_vertical, "splits": {}}
        for split in ("train", "val"):
            images = sorted((source / "images" / split).glob("*.png"))
            split_report = {"images": 0, "boxes": 0, "negative_images": 0}
            for image_path in images:
                label_path = source / "labels" / split / f"{image_path.stem}.txt"
                if not label_path.is_file():
                    raise ValueError(f"Missing label: {label_path}")
                labels = parse_unity_labels(label_path, source_classes, flip_vertical)
                with Image.open(image_path) as loaded:
                    image = loaded.convert("RGB")
                    if flip_vertical:
                        image = ImageOps.flip(image)
                    basename = f"auto_{source.name}_{split}_{image_path.stem}"
                    copy_sample(image, labels, output, split, basename)
                split_report["images"] += 1
                split_report["boxes"] += len(labels)
                split_report["negative_images"] += int(not labels)
            source_report["splits"][split] = split_report
        report["unity_sources"][source.name] = source_report


def manual_shapes(label_path: Path, width: int, height: int) -> list[tuple[int, float, float, float, float]]:
    data = json.loads(label_path.read_text(encoding="utf-8"))
    labels = []
    for shape in data.get("shapes", []):
        name = shape.get("label")
        if name not in CLASS_IDS:
            raise ValueError(f"{label_path}: unknown class {name!r}")
        points = shape.get("points", [])
        if shape.get("shape_type") != "rectangle" or len(points) < 2:
            raise ValueError(f"{label_path}: expected rectangle, got {shape.get('shape_type')!r}")
        xs = [float(point[0]) for point in points]
        ys = [float(point[1]) for point in points]
        x1 = min(max(min(xs), 0.0), float(width))
        x2 = min(max(max(xs), 0.0), float(width))
        y1 = min(max(min(ys), 0.0), float(height))
        y2 = min(max(max(ys), 0.0), float(height))
        box_width, box_height = x2 - x1, y2 - y1
        # A sub-four-pixel annotation carries no reliable visual detail and can
        # become degenerate after integer resizing.
        if box_width < 4.0 or box_height < 4.0:
            continue
        labels.append((
            CLASS_IDS[name],
            ((x1 + x2) / 2.0) / width,
            ((y1 + y2) / 2.0) / height,
            box_width / width,
            box_height / height,
        ))
    return labels


def choose_duplicate(files: list[Path], main_class: str) -> Path:
    def rank(image_path: Path) -> tuple[int, int, str]:
        label_path = image_path.parent.parent / "labels" / f"{image_path.stem}.json"
        data = json.loads(label_path.read_text(encoding="utf-8"))
        labels = [shape.get("label") for shape in data.get("shapes", [])]
        # Prefer an annotation containing the session's intended object, then
        # the most complete annotation. The final filename tie-break is stable.
        return (int(main_class in labels), len(labels), image_path.name)

    return max(files, key=rank)


def split_chunks(files: list[Path], chunk_size: int, seed: int) -> dict[str, list[Path]]:
    chunks = [files[index:index + chunk_size] for index in range(0, len(files), chunk_size)]
    random.Random(seed).shuffle(chunks)
    count = len(chunks)
    n_val = max(1, round(count * 0.2))
    n_test = max(1, round(count * 0.1))
    if n_val + n_test >= count:
        n_val, n_test = 1, 1
    result = {
        "val": [path for chunk in chunks[:n_val] for path in chunk],
        "test": [path for chunk in chunks[n_val:n_val + n_test] for path in chunk],
        "train": [path for chunk in chunks[n_val + n_test:] for path in chunk],
    }
    return result


def add_manual_sources(
    manual_root: Path, output: Path, chunk_size: int, seed: int, report: dict,
) -> None:
    seen_global: set[str] = set()
    for session_index, (session_name, main_class) in enumerate(MANUAL_SESSIONS.items()):
        session = manual_root / session_name
        images = sorted((session / "images").glob("*.png"))
        if not images:
            raise ValueError(f"No PNG images in {session / 'images'}")
        groups: dict[str, list[Path]] = defaultdict(list)
        for image_path in images:
            label_path = session / "labels" / f"{image_path.stem}.json"
            if not label_path.is_file():
                raise ValueError(f"Missing manual label: {label_path}")
            groups[image_digest(image_path)].append(image_path)

        selected = []
        for digest, duplicates in sorted(groups.items()):
            if digest in seen_global:
                continue
            seen_global.add(digest)
            selected.append(choose_duplicate(duplicates, main_class))
        selected.sort(key=lambda path: path.name)
        split_files = split_chunks(selected, chunk_size, seed + session_index)
        session_report = {
            "original_images": len(images),
            "deduplicated_images": len(selected),
            "exact_duplicates_removed": len(images) - len(selected),
            "splits": {},
        }
        for split, split_images in split_files.items():
            split_report = {"images": 0, "boxes": 0, "class_counts": Counter()}
            for image_path in split_images:
                label_path = session / "labels" / f"{image_path.stem}.json"
                with Image.open(image_path) as loaded:
                    image = loaded.convert("RGB")
                    labels = manual_shapes(label_path, image.width, image.height)
                    basename = f"manual_{session_name}_{image_path.stem}"
                    copy_sample(image, labels, output, split, basename)
                split_report["images"] += 1
                split_report["boxes"] += len(labels)
                for class_id, *_ in labels:
                    split_report["class_counts"][CLASSES[class_id]] += 1
            split_report["class_counts"] = dict(split_report["class_counts"])
            session_report["splits"][split] = split_report
        report["manual_sessions"][session_name] = session_report


def summarize_output(output: Path, report: dict) -> None:
    totals = {}
    for split in SPLITS:
        image_files = sorted((output / "images" / split).glob("*.png"))
        label_files = sorted((output / "labels" / split).glob("*.txt"))
        if {path.stem for path in image_files} != {path.stem for path in label_files}:
            raise ValueError(f"Output image/label mismatch in {split}")
        class_counts = Counter()
        negative_images = 0
        for label_file in label_files:
            lines = [line for line in label_file.read_text().splitlines() if line.strip()]
            negative_images += int(not lines)
            for line in lines:
                class_counts[CLASSES[int(line.split()[0])]] += 1
        totals[split] = {
            "images": len(image_files),
            "boxes": sum(class_counts.values()),
            "negative_images": negative_images,
            "class_counts": dict(class_counts),
        }
    report["output"] = totals


def ensure_test_class_coverage(output: Path) -> list[str]:
    """Move the fewest validation samples needed to cover every test class."""
    test_classes = set()
    for label_file in (output / "labels" / "test").glob("*.txt"):
        test_classes.update(
            int(line.split()[0])
            for line in label_file.read_text().splitlines()
            if line.strip()
        )
    missing = set(range(len(CLASSES))) - test_classes
    moved = []
    while missing:
        candidates = []
        for label_file in (output / "labels" / "val").glob("*.txt"):
            present = {
                int(line.split()[0])
                for line in label_file.read_text().splitlines()
                if line.strip()
            }
            gain = len(present & missing)
            if gain:
                candidates.append((gain, label_file.name, label_file, present))
        if not candidates:
            names = ", ".join(CLASSES[index] for index in sorted(missing))
            raise ValueError(f"Cannot provide test coverage for: {names}")
        _, _, label_file, present = max(candidates)
        image_file = output / "images" / "val" / f"{label_file.stem}.png"
        shutil.move(str(label_file), output / "labels" / "test" / label_file.name)
        shutil.move(str(image_file), output / "images" / "test" / image_file.name)
        moved.append(label_file.stem)
        missing -= present
    return moved


def main() -> None:
    args = parse_args()
    if args.manual_chunk_size < 1:
        raise SystemExit("--manual-chunk-size must be positive")
    output = args.output.resolve()
    ensure_empty_output(output, args.clean)
    write_metadata(output)
    report = {
        "classes": CLASSES,
        "seed": args.seed,
        "manual_chunk_size": args.manual_chunk_size,
        "unity_sources": {},
        "manual_sessions": {},
    }
    add_unity_sources(args.auto_source, output, set(args.flip_source_name), report)
    add_manual_sources(args.manual_root.resolve(), output, args.manual_chunk_size, args.seed, report)
    report["test_coverage_samples_moved_from_val"] = ensure_test_class_coverage(output)
    summarize_output(output, report)
    report_path = output / "build_report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Dataset: {output}")
    print(f"Metadata: {output / 'dataset.yaml'}")
    print(f"Report: {report_path}")
    for split, values in report["output"].items():
        print(
            f"{split}: images={values['images']} boxes={values['boxes']} "
            f"negative_images={values['negative_images']}"
        )


if __name__ == "__main__":
    main()

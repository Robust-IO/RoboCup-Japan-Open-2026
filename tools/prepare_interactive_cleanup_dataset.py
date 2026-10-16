#!/usr/bin/env python3
"""Normalize Interactive Cleanup Unity exports into one trainable YOLO dataset.

The exporter used before the candidate-set audit wrote sparse IDs from an old
30-class taxonomy.  Current exports use the 13 official candidates in compact
order.  This tool maps either form by *class name*, so v4 can be retained and
combined with later targeted captures without relabelling by hand.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from PIL import Image


CLASSES = [
    "bear_doll", "canned_juice", "dog_doll", "filled_ketchup",
    "filled_plastic_bottle", "hourglass", "rabbit_doll", "sauce",
    "soysauce", "spray_bottle", "sugar", "toy_penguin", "tumbler",
]
CLASS_IDS = {name: index for index, name in enumerate(CLASSES)}
# Pillow before 9.1 exposes the resampling constants directly on Image.
LANCZOS = getattr(getattr(Image, "Resampling", Image), "LANCZOS")


def read_classes(source: Path) -> list[str]:
    file = source / "classes.txt"
    if not file.is_file():
        raise ValueError(f"Missing {file}")
    names = [line.strip() for line in file.read_text(encoding="utf-8-sig").splitlines()]
    if not names or any(not name for name in names):
        raise ValueError(f"Invalid class list: {file}")
    return names


def remap_labels(label_file: Path, source_classes: list[str]) -> list[str]:
    result: list[str] = []
    for line_number, line in enumerate(label_file.read_text().splitlines(), start=1):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{label_file}:{line_number}: expected 5 fields")
        old_id = int(fields[0])
        if old_id < 0 or old_id >= len(source_classes):
            raise ValueError(f"{label_file}:{line_number}: invalid class ID {old_id}")
        name = source_classes[old_id]
        if name not in CLASS_IDS:
            # A generic old dataset may contain classes that are not candidates
            # in the current Interactive Cleanup configuration.
            continue
        x, y, width, height = (float(value) for value in fields[1:])
        if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 and
                0.0 < width <= 1.0 and 0.0 < height <= 1.0):
            raise ValueError(f"{label_file}:{line_number}: invalid normalized box")
        result.append(f"{CLASS_IDS[name]} {x:.6f} {y:.6f} {width:.6f} {height:.6f}")
    return result


def write_metadata(output: Path) -> None:
    (output / "classes.txt").write_text("\n".join(CLASSES) + "\n", encoding="utf-8")
    lines = ["path: .", "train: images/train", "val: images/val", "names:"]
    lines.extend(f"  {index}: {name}" for index, name in enumerate(CLASSES))
    (output / "data.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", action="append", type=Path, required=True,
                        help="Unity export directory; may be specified more than once")
    parser.add_argument("--output", type=Path, required=True,
                        help="New normalized YOLO dataset directory")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--overwrite", action="store_true",
                        help="Delete an existing output directory before writing")
    args = parser.parse_args()

    output = args.output.resolve()
    if output.exists():
        if not args.overwrite:
            raise SystemExit(f"Output exists: {output}. Use --overwrite or choose a new directory.")
        shutil.rmtree(output)
    for split in ("train", "val"):
        (output / "images" / split).mkdir(parents=True)
        (output / "labels" / split).mkdir(parents=True)
    write_metadata(output)

    output_index = {"train": 0, "val": 0}
    class_counts = [0] * len(CLASSES)
    skipped = 0
    for source in args.source:
        source = source.resolve()
        source_classes = read_classes(source)
        for split in ("train", "val"):
            image_dir = source / "images" / split
            label_dir = source / "labels" / split
            for image_file in sorted(image_dir.glob("*.png")):
                label_file = label_dir / f"{image_file.stem}.txt"
                if not label_file.is_file():
                    raise ValueError(f"Missing label for {image_file}")
                labels = remap_labels(label_file, source_classes)
                if not labels:
                    skipped += 1
                    continue

                destination_name = f"{output_index[split]:06d}"
                output_index[split] += 1
                image_destination = output / "images" / split / f"{destination_name}.png"
                label_destination = output / "labels" / split / f"{destination_name}.txt"
                with Image.open(image_file) as image:
                    image = image.convert("RGB")
                    if image.size != (args.width, args.height):
                        image = image.resize((args.width, args.height), LANCZOS)
                    image.save(image_destination, "PNG")
                label_destination.write_text("\n".join(labels) + "\n", encoding="utf-8")
                for label in labels:
                    class_counts[int(label.split()[0])] += 1

    print(f"Output: {output}")
    print(f"train images: {output_index['train']}")
    print(f"val images: {output_index['val']}")
    print(f"skipped images with no current-task candidate: {skipped}")
    for name, count in zip(CLASSES, class_counts):
        print(f"{name}: {count}")


if __name__ == "__main__":
    main()

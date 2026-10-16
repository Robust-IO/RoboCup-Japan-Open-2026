#!/usr/bin/env python3
"""Validate Unity pilot captures; produce detection + conservative YOLO polygons.

Exact instance PNGs remain the source of truth. Hole contours are connected by
retraced bridges into one contour. Validate the serialized contour AND the
installed Ultralytics resampling/rasterization at IoU >= 0.95. Disconnected
components remain conservative; never drop a visible instance from a frame.
"""
import argparse
import hashlib
import json
from importlib.metadata import version
from pathlib import Path
import shutil

import cv2
import numpy as np

NAMES = ['apple', 'canned_juice', 'rabbit_doll', 'pink_cup', 'white_cup']
GLOBAL_IDS = [0, 3, 15, 14, 26]
ALLOW_DISCONNECTED = False
CONVERTER_VERSION = '2.1-hole-bridges'


def hole_contour(binary):
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        raise ValueError('Empty mask')
    parents = hierarchy[0, :, 3]
    roots = np.flatnonzero(parents == -1)
    if len(roots) != 1 and not ALLOW_DISCONNECTED:
        raise ValueError('Disconnected instance components require separate handling')
    def join(outer, inner):
        distances = ((outer[:, None, :].astype(float) - inner[None, :, :]) ** 2).sum(axis=2)
        oi, ii = np.unravel_index(distances.argmin(), distances.shape)
        loop = np.concatenate([inner[ii:], inner[:ii], inner[ii:ii+1]])
        return np.concatenate([outer[:oi+1], loop, outer[oi:oi+1], outer[oi+1:]])

    components = []
    # Keep original contour vertices. A bridge traversed in both directions
    # permits OpenCV even-odd filling to preserve the hole interior.
    for root in roots:
        outer = contours[int(root)][:, 0, :]
        for child in np.flatnonzero(parents == root):
            outer = join(outer, contours[int(child)][:, 0, :])
        components.append(outer)
    outer = components[0]
    for component in components[1:]:
        outer = join(outer, component)
    if len(outer) < 3:
        raise ValueError('Degenerate polygon')
    return outer


def contour_iou(binary, points):
    raster = np.zeros_like(binary)
    cv2.fillPoly(raster, [points.astype(np.int32)], 1)
    return float(np.logical_and(raster, binary).sum() / np.logical_or(raster, binary).sum())


def training_iou(binary, points):
    # Use the installed trainer, not a guessed equivalent. Any package upgrade
    # must pass this check again. This is before geometric augmentation.
    from ultralytics.utils.ops import resample_segments
    from ultralytics.data.utils import polygon2mask
    sampled = resample_segments([points.astype(np.float32).copy()], n=max(1000, len(points)+1))[0]
    raster = polygon2mask(binary.shape, [sampled.reshape(-1)], downsample_ratio=1)
    return float(np.logical_and(raster, binary).sum() / np.logical_or(raster, binary).sum())


def decode_ids(rgb):
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('Mask must be a 3-channel RGB PNG')
    if not np.all((rgb <= 5) | (rgb >= 250)):
        raise ValueError('Nonbinary mask colors: check shader, postprocessing and MSAA')
    bits = (rgb >= 250).astype(np.uint8)
    ids = bits[:, :, 0] + 2 * bits[:, :, 1] + 4 * bits[:, :, 2]
    if ids.max() > len(NAMES):
        raise ValueError(f'Unknown instance ID (expected 0..{len(NAMES)})')
    return ids


def annotations(ids, instances, min_pixels=32):
    h, w = ids.shape
    declared = set()
    segments, boxes = [], []
    for instance in instances:
        iid, cls = instance['instanceId'], instance['classId']
        if not isinstance(cls, int) or not 0 <= cls < len(NAMES) or iid != cls + 1 or iid in declared:
            raise ValueError('Invalid or duplicate instance/class ID')
        if instance['className'] != NAMES[cls] or instance['globalClassId'] != GLOBAL_IDS[cls]:
            raise ValueError('Class mapping mismatch')
        declared.add(iid)
        binary = (ids == iid).astype(np.uint8)
        area = int(binary.sum())
        if area == 0:  # Fully occluded / off screen: never emit an invisible bbox.
            continue
        if area < min_pixels:
            raise ValueError(f'Visible instance {iid} too small ({area} pixels); quarantine whole frame')
        contour = hole_contour(binary)
        # Pixel centers prevent decimal serialization from moving integer-edge
        # vertices a full pixel when the trainer converts float32 to int32.
        points = (contour.astype(float) + .5) / np.array([w, h])
        line = str(cls) + ' ' + ' '.join(f'{v:.8f}' for v in points.flat)
        decoded = np.array(line.split()[1:], dtype=np.float32).reshape(-1, 2) * np.array([w, h], dtype=np.float32)
        native_iou = contour_iou(binary, decoded)
        train_iou = training_iou(binary, decoded)
        if min(native_iou, train_iou) < .95:
            raise ValueError(f'Instance {iid}: contour fidelity below 0.95 (serialized={native_iou:.3f}, trainer={train_iou:.3f})')
        segments.append(line)
        ys, xs = np.nonzero(binary)
        x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
        coords = [(x0+x1)/(2*w), (y0+y1)/(2*h), (x1-x0)/w, (y1-y0)/h]
        boxes.append(str(cls) + ' ' + ' '.join(f'{v:.8f}' for v in coords))
    if set(np.unique(ids)) - {0} - declared:
        raise ValueError('Visible mask ID missing from metadata')
    return segments, boxes


def read_rgb(path):
    # imdecode supports Windows paths containing non-ASCII characters, too.
    image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f'Unreadable image: {path}')
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def within(root, relative):
    p = (root / relative).resolve()
    if not p.is_relative_to(root.resolve()):
        raise ValueError('Image path escapes batch directory')
    return p


def inspect_batch(root):
    root = Path(root).resolve()
    batch = json.loads((root / 'batch.json').read_text(encoding='utf-8-sig'))
    if batch.get('schemaVersion') != 1:
        raise ValueError('Unsupported batch schema')
    if [s['name'] for s in batch['sources']] != NAMES or [s['globalClassId'] for s in batch['sources']] != GLOBAL_IDS:
        raise ValueError('Unexpected source class registry')
    frames = sorted((root / 'frames').glob('*.json'))
    if not frames:
        raise ValueError(f'No completed frames: {root}')
    return root, batch, frames


def prepare(splits, output, inspect_only=False):
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('Output already exists: choose a new directory (never overwritten)')
    batches, seeds = [], set()
    for split, roots in splits.items():
        for root in roots:
            root, batch, frames = inspect_batch(root)
            if batch['seed'] in seeds:
                raise ValueError('Duplicate batch seed across inputs; use independent train/val/test seeds')
            seeds.add(batch['seed'])
            batches.append((split, root, batch, frames))
    if not batches or (not inspect_only and (not splits.get('train') or not splits.get('val'))):
        raise ValueError('Separate train and val batches are required')
    output.mkdir(parents=True)
    report = {'schemaVersion': 1, 'converterVersion': CONVERTER_VERSION, 'minContourIoU': .95,
              'disconnectedPolicy': 'retraced-bridges-IoU-gated' if ALLOW_DISCONNECTED else 'reject',
              'ultralyticsVersion': version('ultralytics'),
              'validation': 'serialized float32 and installed Ultralytics resampling, full resolution, before augmentation',
              'accepted': {}, 'rejected': [], 'classes': NAMES,
              'classCounts': {}, 'sources': [], 'note': 'Synthetic pilot only; not a real-camera evaluation.'}
    seen_images = {}  # Detect exact duplicate RGB across splits, beyond seed checks.
    for split, root, batch, frames in batches:
        report['sources'].append({'split': split, 'path': str(root), 'seed': batch['seed'],
                                  'completed': len(frames), 'requested': batch['count']})
        report['accepted'].setdefault(split, 0)
        report['classCounts'].setdefault(split, [0] * len(NAMES))
        for folder in ['images', 'labels', 'labels_detect', 'masks', 'metadata', 'previews']:
            (output / folder / split).mkdir(parents=True, exist_ok=True)
        for fp in frames:
            try:
                frame = json.loads(fp.read_text(encoding='utf-8-sig'))
                if frame.get('schemaVersion') != 1 or frame['seed'] != batch['seed'] or frame['group'] != 'seed_' + str(batch['seed']):
                    raise ValueError('Frame schema/seed/group mismatch')
                if frame['index'] < 0 or frame['index'] >= batch['count'] or fp.stem != f"{frame['index']:06d}":
                    raise ValueError('Invalid frame index/filename')
                rgb_path, mask_path = within(root, frame['rgb']), within(root, frame['mask'])
                rgb, mask = read_rgb(rgb_path), read_rgb(mask_path)
                expected = (batch['height'], batch['width'], 3)
                if rgb.shape != expected or mask.shape != expected or frame['width'] != batch['width'] or frame['height'] != batch['height']:
                    raise ValueError('RGB/mask/metadata resolution mismatch')
                ids = decode_ids(mask)
                segments, boxes = annotations(ids, frame['instances'])
                if frame['negative'] and (frame['instances'] or ids.any()):
                    raise ValueError('Negative frame contains instances')
                if not frame['negative'] and not segments:
                    raise ValueError('Positive frame has no visible objects; check camera/shader')
                digest = hashlib.sha256(rgb.tobytes()).hexdigest()
                if digest in seen_images:
                    raise ValueError('Duplicate RGB image of ' + seen_images[digest])
                seen_images[digest] = split + '/' + fp.name
                stem = f"s{batch['seed']}_{fp.stem}"
                shutil.copy2(rgb_path, output / 'images' / split / (stem + '.png'))
                shutil.copy2(mask_path, output / 'masks' / split / (stem + '.png'))
                shutil.copy2(fp, output / 'metadata' / split / (stem + '.json'))
                for folder, lines in [('labels', segments), ('labels_detect', boxes)]:
                    (output / folder / split / (stem + '.txt')).write_text('\n'.join(lines) + ('\n' if lines else ''), encoding='utf-8')
                if report['accepted'][split] < 50:
                    overlay = rgb.copy()
                    visible = ids > 0
                    overlay[visible] = (rgb[visible] * .55 + mask[visible] * .45).astype(np.uint8)
                    for line in boxes:
                        cls, x, y, bw, bh = map(float, line.split())
                        h, w = ids.shape
                        p0, p1 = (round((x-bw/2)*w), round((y-bh/2)*h)), (round((x+bw/2)*w), round((y+bh/2)*h))
                        cv2.rectangle(overlay, p0, p1, (0, 255, 0), 1)
                        cv2.putText(overlay, NAMES[int(cls)], p0, cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 0), 1)
                    cv2.imencode('.png', cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))[1].tofile(output / 'previews' / split / (stem + '.png'))
                report['accepted'][split] += 1
                for line in segments:
                    report['classCounts'][split][int(line.split()[0])] += 1
            except (ValueError, KeyError, OSError, TypeError) as exc:
                report['rejected'].append({'frame': str(fp), 'reason': str(exc)})
    (output / 'qa_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    if inspect_only:
        return report
    for split in splits:
        if splits[split] and (report['accepted'].get(split, 0) == 0 or any(n == 0 for n in report['classCounts'][split])):
            raise ValueError(f'{split}: no usable frames or missing classes; inspect {output}/qa_report.json. No dataset YAML was created.')
    content = 'path: ' + json.dumps(str(output)) + '\ntrain: images/train\nval: images/val\n'
    if splits.get('test'):
        content += 'test: images/test\n'
    content += 'names:\n' + ''.join(f'  {i}: {name}\n' for i, name in enumerate(NAMES))
    (output / 'dataset-seg.yaml').write_text(content, encoding='utf-8')
    # Detection dataset uses a separate image/labels tree (YOLO infers labels path).
    detect = output / 'detect'
    for split in splits:
        if not splits[split]:
            continue
        (detect / 'images' / split).mkdir(parents=True)
        shutil.copytree(output / 'labels_detect' / split, detect / 'labels' / split)
        for image in (output / 'images' / split).glob('*.png'):
            try:
                (detect / 'images' / split / image.name).hardlink_to(image)
            except OSError:
                shutil.copy2(image, detect / 'images' / split / image.name)
    (output / 'dataset-detect.yaml').write_text(content.replace(json.dumps(str(output)), json.dumps(str(detect)), 1), encoding='utf-8')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inspect', type=Path, help='QA/overlays for one batch; do not create training YAML')
    p.add_argument('--train', nargs='+', default=[], type=Path)
    p.add_argument('--val', nargs='+', default=[], type=Path)
    p.add_argument('--test', nargs='+', default=[], type=Path)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    if args.inspect:
        if args.train or args.val or args.test:
            p.error('--inspect cannot be combined with dataset split inputs')
        report = prepare({'inspect': [args.inspect]}, args.output, inspect_only=True)
    else:
        if not args.train or not args.val:
            p.error('Use --inspect BATCH, or provide both --train and --val')
        report = prepare({s: getattr(args, s) for s in ['train', 'val', 'test']}, args.output)
    print(json.dumps({'accepted': report['accepted'], 'rejected': len(report['rejected']), 'classCounts': report['classCounts']}, indent=2))


if __name__ == '__main__':
    main()

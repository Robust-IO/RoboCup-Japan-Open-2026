"""Conservative offline same-class NMS. Never overwrites input predictions."""
import argparse
import copy
import json
import base64
import html
from pathlib import Path
from audit_detection_duplicates import iou


def deduplicate(detections, threshold=.85):
    if not 0 < threshold <= 1:
        raise ValueError('threshold must be in (0, 1]')
    kept, removed = [], []
    for index in sorted(range(len(detections)), key=lambda i: (-detections[i]['confidence'], i)):
        candidate = detections[index]
        match = next((k for k in kept if detections[k]['name'] == candidate['name']
                      and iou(detections[k]['xyxy'], candidate['xyxy']) >= threshold), None)
        if match is None:
            kept.append(index)
        else:
            removed.append(dict(index=index, retained_index=match,
                                iou=iou(candidate['xyxy'], detections[match]['xyxy'])))
    return sorted(kept), removed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--html', type=Path)
    args = parser.parse_args()
    original = json.loads(args.input.read_text())
    output = copy.deepcopy(original)
    totals = {}
    for case in output['cases']:
        for result in case['results']:
            ds = result['detections']
            kept, removed = deduplicate(ds)
            result['offline_dedup'] = dict(threshold=.85, kept_indices=kept, removed=removed)
            key = f"{result['model']}-{result['imgsz']}"
            s = totals.setdefault(key, dict(frames=0, changed_frames=0, before=0, after=0))
            s['frames'] += 1
            s['changed_frames'] += bool(removed)
            s['before'] += len(ds)
            s['after'] += len(kept)
            assert {d['name'] for d in ds} == {ds[i]['name'] for i in kept}
    output['dedup_summary'] = totals
    output['dedup_limitations'] = 'Offline only; raw detections retained. No cross-class suppression. Class presence preservation does not prove instance recall preservation; visual review required.'
    with args.output.open('x') as f:
        json.dump(output, f, indent=2)
    if args.html:
        from PIL import Image
        page = ['<!doctype html><meta charset="utf-8"><title>重复框离线核查</title><style>body{font:16px sans-serif;margin:24px}section{display:flex;gap:16px}article{width:48%}svg{width:100%}</style><h1>同类重复框：处理前后</h1><p>仅展示新模型发生变化的图。IoU≥0.85，同类保留高分框；不同类别不删除。绿色为保留框，红色虚线为拟删除框。未经逐框真值验证，不能据此宣称召回率不变。</p>']
        for c in output['cases']:
            for r in c['results']:
                if r['model'] != 'new' or not r['offline_dedup']['removed']:
                    continue
                source = Path(c['source'])
                with Image.open(source) as im:
                    w, h = im.size
                raw = base64.b64encode(source.read_bytes()).decode()
                mime = 'image/png' if source.suffix.lower() == '.png' else 'image/jpeg'
                page.append(f'<h2>{c["id"]} / {r["imgsz"]}</h2><section>')
                kept = r['offline_dedup']['kept_indices']
                for after in [False, True]:
                    page.append(f'<article><h3>{"处理后" if after else "处理前（标出拟删除框）"}</h3><svg viewBox="0 0 {w} {h}"><image href="data:{mime};base64,{raw}" width="{w}" height="{h}"/>')
                    for i, d in enumerate(r['detections']):
                        if after and i not in kept:
                            continue
                        x, y, x2, y2 = d['xyxy']
                        color = 'lime' if i in kept else 'red'
                        dash = '' if i in kept else 'stroke-dasharray="4 3"'
                        label = html.escape(f'{i}: {d["name"]} {d["confidence"]:.3f}')
                        page.append(f'<rect x="{x}" y="{y}" width="{x2-x}" height="{y2-y}" stroke="{color}" fill="none" {dash}/><text x="{x}" y="{max(12,y-3)}" fill="{color}" stroke="black" stroke-width=".2" font-size="10">{label}</text>')
                    page.append('</svg></article>')
                page.append('</section>')
        with args.html.open('x', encoding='utf-8') as f:
            f.write('\n'.join(page))
    print(json.dumps(totals, indent=2))


if __name__ == '__main__':
    main()

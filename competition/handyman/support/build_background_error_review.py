"""Build a targeted review of existing full-frame predictions, without relabeling."""
import base64
import html
import json
from pathlib import Path
from PIL import Image

ROOT = Path('/__HANDYMAN_PACKAGE__/runtime')
NAMES = {'tumbler', 'pink_cup', 'spray_bottle', 'empty_ketchup', 'nursing_bottle', 'rabbit_doll'}


def main():
    report = json.loads((ROOT/'live30-pose6-recheck-20261005-v1/report.json').read_text())
    rows = []
    for case in report['cases']:
        for result in case['results']:
            if result['model'] != 'new':
                continue
            for i, d in enumerate(result['detections']):
                if d['name'] in NAMES:
                    rows.append(dict(id=f'{case["id"]}-{result["imgsz"]}-{i}', case=case['id'],
                                     source=case['source'], source_sha256=case['sha256'],
                                     imgsz=result['imgsz'], detection=d))
    out = ROOT/'background-errors-review-v1'
    out.mkdir(exist_ok=False)
    (out/'manifest.json').write_text(json.dumps(dict(rows=rows, weights=report['weights']['new'],
        policy='Historical regression only. No automatic negative labels; no training ingestion.'), indent=2))
    page = ['<!doctype html><meta charset="utf-8"><title>背景误检定向核查</title><style>body{font:16px sans-serif;margin:24px;background:#eee}article{background:white;padding:15px;margin:15px 0}svg{width:640px;max-width:100%}select,button{font-size:18px;padding:8px}</style><h1>背景误检定向核查</h1><p>每张图只突出一个待核查框。请按框中的实际内容判断，不要仅看类别名称。同图不同输入尺寸独立显示。未确认项保留“待判断”。这批图片仅作回归，不进入训练。</p><button onclick="download()">导出核查结果</button>']
    for row in rows:
        source = Path(row['source'])
        with Image.open(source) as im:
            w, h = im.size
        raw = base64.b64encode(source.read_bytes()).decode()
        mime = 'image/png' if source.suffix.lower() == '.png' else 'image/jpeg'
        d = row['detection']; x, y, x2, y2 = d['xyxy']
        page.append(f'<article><h3>{row["id"]} · {html.escape(d["name"])} · {d["confidence"]:.1%}</h3><svg viewBox="0 0 {w} {h}"><image href="data:{mime};base64,{raw}" width="{w}" height="{h}"/><rect x="{x}" y="{y}" width="{x2-x}" height="{y2-y}" fill="none" stroke="red" stroke-width="2"/></svg><p><select data-id="{row["id"]}"><option value="pending">待判断</option><option value="correct">真实目标，类别正确</option><option value="background">背景／非30类物品误检</option><option value="wrong_class">真实30类物品，但类别错误</option><option value="self">机器人自身</option><option value="uncertain">无法确定</option></select> <input data-note="{row["id"]}" placeholder="如：墙上五角星、花瓶、叶子"></p></article>')
    page.append('''<script>
const key='handyman-background-errors-review-v1';
let saved=JSON.parse(localStorage.getItem(key)||'{}');
for(const el of document.querySelectorAll('select,input')){const id=el.dataset.id||el.dataset.note;const field=el.tagName==='SELECT'?'decision':'note';el.value=(saved[id]||{})[field]|| (field==='decision'?'pending':'');el.onchange=()=>{(saved[id]??={})[field]=el.value;localStorage.setItem(key,JSON.stringify(saved));};}
function download(){const rows=[...document.querySelectorAll('select')].map(el=>({id:el.dataset.id,decision:el.value,note:document.querySelector('[data-note="'+el.dataset.id+'"]').value}));const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify({schema:'background-errors-review-v1',rows},null,2)],{type:'application/json'}));a.download='background-errors-human-review.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}
</script>''')
    (out/'index.html').write_text('\n'.join(page), encoding='utf-8')
    print(json.dumps(dict(output=str(out),review_boxes=len(rows),frames=len({r['case'] for r in rows}))))


if __name__ == '__main__':
    main()

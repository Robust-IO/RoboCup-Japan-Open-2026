from competition_paths import is_run_path
from pathlib import Path
import json, zipfile, hashlib
root=Path('/__HANDYMAN_PACKAGE__/runtime')
bundle=root/'live30-evidence-20261004-v1.zip'
manifest=[]
with zipfile.ZipFile(bundle,'x',zipfile.ZIP_DEFLATED) as z:
    added=set()
    def add(p):
        if p.is_file() and is_run_path(p) and str(p) not in added:
            z.write(p,str(p.relative_to(root))); added.add(str(p))
    for group in ['live30-20261003','live30-20261004','salt-pepper-live-20261003']:
        for p in sorted((root/group).rglob('predictions.json')):
            add(p)
            rows=json.loads(p.read_text())
            manifest.append({'file':str(p.relative_to(root)),'rows':len(rows) if isinstance(rows,list) else 'dict'})
            for pic in p.parent.glob('*.jpg'): add(pic)
            for pic in p.parent.glob('*.png'): add(pic)
            for r in rows if isinstance(rows,list) else []:
                for k in ['source','image','image_path']:
                    if isinstance(r.get(k),str):
                        pic=Path(r[k]); add(pic); add(pic.with_suffix('.json'))
        for p in (root/group).glob('*.txt'): add(p)
    for n in [40,42,44,64,66,68,116,118,120]:
        add(root/'salt-pepper-live-20261003/capture02'/f'{n:06}.png')
    z.writestr('manifest.json',json.dumps(manifest,indent=2))
print(json.dumps({'path':str(bundle),'files':len(added),'comparisons':len(manifest),'bytes':bundle.stat().st_size}))

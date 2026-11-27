import json
from pathlib import Path
root=Path('/__HANDYMAN_PACKAGE__/runtime/pepper-compact-realign-open-continuation-20261026-01')
rows=[]
for p in sorted((root/'grasp/captures').glob('*/raw-detector-result.json')):
 d=json.loads(p.read_text());rows.append(dict(capture=str(p.parent),detections=[x for x in d.get('detections',[]) if x['name']=='ground_pepper']))
print(json.dumps(rows))

import json,sys
from pathlib import Path
import numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');sys.path[:0]=[str(b),'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1']
from source_table_front import observed_front
root=b/'cigarette-right-shift-20261101-194019';audit=json.loads((root/'current-depth-audit.json').read_text());cap=root/'captures/after-right-shift';s=json.loads((cap/'snapshot.json').read_text());d=np.load(cap/'depth_m.npy')
cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==cfg['source_table_id'])
t,proof=observed_front(d,s,table,audit['plane']);print(json.dumps(dict(table=t,proof=proof)));(root/'observed-front-audit.json').write_text(json.dumps(dict(table=t,proof=proof),indent=2))

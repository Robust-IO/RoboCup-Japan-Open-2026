"""Read-only input; new D-drive output to measure lossless capture write cost."""
import json
from pathlib import Path
import time
import statistics
import cv2
import numpy as np

source=Path('/__HANDYMAN_PACKAGE__/runtime/apple-capture-budget-r13-source-readonly-20261018-01/source-observe/000010')
out=Path('/__HANDYMAN_PACKAGE__/runtime/apple-capture-budget-r13-save-benchmark-20261018-01');out.mkdir(exist_ok=False)
rgb=cv2.imread(str(source/'rgb.png'));hand=cv2.imread(str(source/'hand.png'));depth=np.load(source/'depth_m.npy',allow_pickle=False)
snapshot=json.loads((source/'snapshot.json').read_text());rows=[]
for level in (1,0):
    for i in range(5):
        folder=out/f'level{level}-{i}';folder.mkdir();row=dict(level=level)
        for name,im in (('rgb',rgb),('hand',hand)):
            start=time.perf_counter();cv2.imwrite(str(folder/(name+'.png')),im,[cv2.IMWRITE_PNG_COMPRESSION,level])
            row[name+'_s']=time.perf_counter()-start;row[name+'_bytes']=(folder/(name+'.png')).stat().st_size
            if not np.array_equal(cv2.imread(str(folder/(name+'.png'))),im):raise ValueError('lossless_pixel_mismatch')
        start=time.perf_counter();np.save(folder/'depth_m.npy',depth);row['depth_s']=time.perf_counter()-start
        start=time.perf_counter();(folder/'snapshot.json').write_text(json.dumps(snapshot,indent=2));row['snapshot_s']=time.perf_counter()-start
        rows.append(row)
report=dict(rows=rows,medians={str(level):{key:statistics.median(r[key] for r in rows if r['level']==level)
    for key in ('rgb_s','hand_s','depth_s','snapshot_s','rgb_bytes','hand_bytes')} for level in (1,0)},
    input_unchanged=True,lossless_pixels_verified=True,robot_commands_sent=False)
with (out/'result.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)

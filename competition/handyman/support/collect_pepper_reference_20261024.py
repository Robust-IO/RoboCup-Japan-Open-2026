"""Collect saved RGBD only; no ROS imports or live robot access."""
import hashlib
import json
import tarfile
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime')
names=['can-centerline-baseline-depth-visible-20261024-07','can-centered-continuous-live-20261024-08',
    'can-centered-initial-return-live-20261024-09','can-extra-seat35-live-20261024-03',
    'can-upright-calibrated-live-20261024-01','can-centerline-seat35-visible-20261024-05']
out=base/'pepper-offline-reference-v1-20261024';out.mkdir(exist_ok=False)
rows=[]
with tarfile.open(out/'frames.tar.gz','x:gz') as archive:
    for index,name in enumerate(names):
        capture=base/name/'grasp/captures/identity-02'
        if not capture.is_dir():continue
        raw=json.loads((capture/'raw-detector-result.json').read_text())
        detections=[d for d in raw['detections'] if d['name']=='ground_pepper' and d['confidence']>=.8]
        if len(detections)!=1:continue
        files=['rgb.png','depth_m.npy','snapshot.json','raw-detector-result.json']
        hashes={n:hashlib.sha256((capture/n).read_bytes()).hexdigest() for n in files}
        for n in files:archive.add(capture/n,arcname=f'{index:02d}/'+n)
        rows.append(dict(id=f'{index:02d}',capture=str(capture),detection=detections[0],hashes=hashes))
(out/'manifest.json').write_text(json.dumps(dict(class_name='ground_pepper',class_id=11,
    source='saved_actual_registered_rgbd_not_unity_ground_truth',samples=rows,
    robot_commands_sent=False,model_changed=False),indent=2))
print(json.dumps(dict(directory=str(out),frames=len(rows),archive_sha256=hashlib.sha256((out/'frames.tar.gz').read_bytes()).hexdigest())))

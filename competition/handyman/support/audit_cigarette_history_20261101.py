"""Read-only comparison of actual successful and rejected source records."""
import json,sys,hashlib,zipfile
from pathlib import Path
import cv2,numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');sys.path[:0]=[str(b),'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1']
from flat_box_geometry import estimate
from source_table_front import observed_front
from can_destination_surface import measure
from apple_transfer_auto_plan import map_pose,map_point
cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==cfg['source_table_id'])
rows=[]
for root in sorted(list(b.glob('cigarette-grasp-20261101-*'))+list(b.glob('cigarette-contact-20261101-*'))):
 if not root.is_dir() or not (root/'result.json').exists():continue
 result=json.loads((root/'result.json').read_text());keys=('status','reason','source_table_verified','source_model_detection_used','source_identity','operator_confirmed_test_object','grasp_success_verified','approach_commands_sent','close_commands_sent','elapsed_s','continued_from_open_predecessor')
 row=dict(run=root.name,result={k:result[k] for k in keys if k in result})
 receipt=b/(root.name+'-launch.json')
 if receipt.exists():row['command']=json.loads(receipt.read_text()).get('command')
 caps=[root/'captures/source'] if (root/'captures/source/snapshot.json').exists() else []
 if (root/'closure-contact.json').exists():
  closure=json.loads((root/'closure-contact.json').read_text());path=closure.get('before_capture')
  if path:caps.append(Path(path))
 frames=[]
 for cap in caps:
  try:
   s=json.loads((cap/'snapshot.json').read_text());depth=np.load(cap/'depth_m.npy');rgb=cv2.cvtColor(cv2.imread(str(cap/'rgb.png')),cv2.COLOR_BGR2RGB)
   frame=dict(capture=str(cap),pose=map_pose(s),base_from_rgb=s['base_from_rgb'],rgb_k=s['rgb_k'])
   plane=measure(depth,s,table)
   try:_,proof=observed_front(depth,s,table,plane);frame['observed_front']=proof
   except Exception as exc:frame['front_reason']=str(exc)
   modelpath=root/'source-geometry.json'
   if modelpath.exists():
    m=json.loads(modelpath.read_text());frame['saved_model']=m;frame['saved_source_center_map']=map_point(s,np.asarray(m['center_m'])).tolist() if cap.name=='source' else None
   dp=cap/'detections.json'
   if dp.exists():
    det=json.loads(dp.read_text())['detections'];frame['targets']=[]
    for d in det:
     if d['name']!='cigarette':continue
     try:
      m=estimate(rgb,depth,s,d);frame['targets'].append(dict(confidence=d['confidence'],center_base=m['center_m'],center_map=map_point(s,np.asarray(m['center_m'])).tolist()))
     except Exception as exc:frame['targets'].append(dict(reason=str(exc)))
   frames.append(frame)
  except Exception as exc:frames.append(dict(capture=str(cap),reason=str(exc)))
 row['frames']=frames;rows.append(row)
report=dict(robot_commands_sent=False,table=table,runs=rows)
(b/'cigarette-history-depth-and-gates-audit-20261101.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
for row in rows:
 print(json.dumps(dict(run=row['run'],result=row['result'],frames=[{k:f[k] for k in ('capture','pose','observed_front','front_reason','targets','saved_source_center_map') if k in f} for f in row['frames']],operator_reference_flag=bool('--operator-confirmed-reference' in str(row.get('command')))),ensure_ascii=False))
archive=b/'backups/before-outcome-policy-20261101-182352/transfer-before-changes.zip'
with zipfile.ZipFile(archive) as z:
 for key in ('active/flat_box_geometry.py','active/flat_box_grasp.py'):
  if key in z.namelist():
   txt=z.read(key).decode();print(json.dumps(dict(backup_file=key,candidate_gate_existed='candidate_requires_base_inside_table' in txt,source_selection_used='select_source(' in txt)))

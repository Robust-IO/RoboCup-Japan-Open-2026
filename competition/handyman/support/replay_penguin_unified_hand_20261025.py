"""Offline replay across archived staged app epochs; never changes snapshots."""
import importlib.util,json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
spec=importlib.util.spec_from_file_location('new_hand','/__HANDYMAN_PACKAGE__/runtime/penguin-unified-hand-candidate-20261025.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'penguin-unified-hand-offline-replay-20261025-01';root.mkdir(exist_ok=False)
prior=base/'penguin-native-transfer-20261025-10';report=json.loads((prior/'result.json').read_text());before=Path(report['close']['before_capture'])
s=json.loads((before/'snapshot.json').read_text());m=json.loads((before/'object-measurement.json').read_text());model=report['source_observations'][-1]['measurement']
ref=g.create_reference(cv2.imread(str(before/'hand.png')),s,m,root/'face.png',model=model)
contact=g.contact_context(model,m,s,ref,report['close']['closure_evidence']['observed_opening_rad'])
groups=[prior/'grasp/captures/014-near-rejected',
 base/'penguin-current-head-lift-test-20261025-01/captures/after-lift-0',
 base/'penguin-head-retraction-test-20261025-01/captures/after-retraction-0',
 base/'penguin-head-carry-test-20261025-01/captures/destination-held-0']
place=json.loads((base/'penguin-head-place-test-20261025-01/result.json').read_text())
groups += [Path(place['release_support']['capture'])]
rows=[]
for folder in groups:
 snapshot=json.loads((folder/'snapshot.json').read_text());image=cv2.imread(str(folder/'hand.png'))
 # These historical trials used distinct ROS application epochs. Only the
 # replay reference is explicitly mapped; production requires one live epoch.
 mapped=dict(ref,tracking_epoch=snapshot['tracking_epoch'])
 evidence=g.verify_hand(image,snapshot,mapped,contact['holding_opening_rad'])
 held=dict(contact,grasp_lift_verified=True,hand_face_reference=mapped)
 estimate=g.estimate_held(g.palm_at_capture(snapshot),snapshot,held,evidence)
 rows.append(dict(capture=str(folder),correlation=evidence['correlation'],pixel_shift=evidence['pixel_shift'],
                  estimated_pose_source=estimate['position_source'],current_depth_measured=estimate['current_head_depth_measured']))
released=Path(place['placed_frames'][0]['capture']);snapshot=json.loads((released/'snapshot.json').read_text())
rejected=False
try:g.verify_hand(cv2.imread(str(released/'hand.png')),snapshot,dict(ref,tracking_epoch=snapshot['tracking_epoch']),contact['holding_opening_rad'])
except ValueError:rejected=True
assert rejected,'released object cannot remain held'
result=dict(robot_commands_sent=False,archived_epoch_mapping_for_offline_replay_only=True,
 template_box=ref['box'],rows=rows,released_hand_rejected=True,all_held_images_verified=True)
(root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)

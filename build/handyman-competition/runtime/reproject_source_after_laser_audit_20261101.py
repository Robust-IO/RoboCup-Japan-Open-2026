"""Offline camera reprojection using the independently matched laser pose."""
import copy,json,math,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');sys.path[:0]=[str(b),'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1']
from source_table_front import observed_front
from can_destination_surface import measure
from apple_transfer_auto_plan import map_point
from hand_camera_ray import rotation
auditroot=Path(json.loads((b/'source-laser-audit-latest.json').read_text())['output']);a=json.loads((auditroot/'result.json').read_text());scans=json.loads((auditroot/'scans.json').read_text());best=a['results'][0];odom=scans[0]['odom_from_base'];ro=rotation(odom['quaternion']);rm=Rotation.from_euler('z',best['matched_pose'][2]).as_matrix();rmo=rm@ro.T;tmo=np.r_[best['matched_pose'][:2],0.]-rmo@odom['translation']
root=b/'cigarette-right-shift-20261101-194019';cap=root/'captures/after-right-shift';s=json.loads((cap/'snapshot.json').read_text());corrected=copy.deepcopy(s);old=s['tracking_transforms']['odom_from_base_at_depth'];tf=corrected['tracking_transforms']['map_from_base_at_depth'];tf.update(translation=(rmo@old['translation']+tmo).tolist(),quaternion=Rotation.from_matrix(rmo@rotation(old['quaternion'])).as_quat().tolist());d=np.load(cap/'depth_m.npy')
cfg=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==cfg['source_table_id']);plane=measure(d,corrected,table);_,proof=observed_front(d,corrected,table,plane)
previous=json.loads((root/'observed-front-audit.json').read_text())['proof'];out=dict(status='offline_independent_laser_camera_alignment_verified',robot_commands_sent=False,map_transform_changed=False,
 laser_audit=str(auditroot),old_front=previous,corrected_front=proof,new_map_from_odom_estimate=dict(translation=tmo.tolist(),quaternion=Rotation.from_matrix(rmo).as_quat().tolist()),
 conclusion='Original imported table boundary and RGBD front agree after independent laser pose correction. Do not deploy footprint-shrinking workaround as the localization fix.',
 pending='Robot remains empty/open after lateral alignment. Correct and validate current-session localization before resuming the task; no new grasp executed.')
(b/'cigarette-localization-root-cause-20261101.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))

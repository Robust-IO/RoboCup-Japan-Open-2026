"""No-motion live validation of correction, fresh recapture and target continuity."""
import argparse,json,time,uuid
from pathlib import Path
import cv2,numpy as np
import rclpy
from apple_transfer_automation import RobotLock,write_json
from flat_box_place import TaskPlaceRuntime
from table_localization_client import localize
from flat_box_geometry import appearance_from_source
from tracked_box_source import reacquire
from can_destination_surface import measure
from source_table_front import observed_front

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();b=Path('/__HANDYMAN_PACKAGE__/runtime');root=a.output
report=dict(status='starting',robot_commands_sent=False);runtime=None
with RobotLock():
 root.mkdir();rclpy.init()
 try:
  runtime=TaskPlaceRuntime(root,'table-localization-probe-'+uuid.uuid4().hex,lambda _:None);runtime.sample(processing_reserve_ns=0);before=runtime.state()
  first=localize(runtime,root,'first');second=localize(runtime,root,'second')
  sample=runtime.sample(processing_reserve_ns=0);capture=runtime.save(sample,'corrected-current')
  old=b/'cigarette-right-shift-20261101-194019';oldm=json.loads((old/'result.json').read_text())['measurement'];oldcap=old/'captures/before-right-shift';olds=json.loads((oldcap/'snapshot.json').read_text());oldrgb=cv2.cvtColor(cv2.imread(str(oldcap/'rgb.png')),cv2.COLOR_BGR2RGB);oldd=np.load(oldcap/'depth_m.npy')
  reference=dict(snapshot=olds,model=oldm,appearance=appearance_from_source(oldrgb,oldd,olds,oldm),model_detection_verified=True,
   source_run='cigarette-full-auto-20261101-193127',source_capture=str(oldcap))
  reference_path=b/'cigarette-current-target-reference-20261101.json';write_json(reference_path,reference)
  cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==cfg['source_table_id'])
  current=reacquire(sample['rgb'],sample['depth'],sample['snapshot'],reference,table);plane=measure(sample['depth'],sample['snapshot'],table);_,front=observed_front(sample['depth'],sample['snapshot'],table,plane)
  if second['correction_applied']:raise ValueError('aligned_pose_unnecessarily_corrected_again')
  report.update(status='automatic_table_localization_live_probe_passed',first=dict(correction_applied=first['correction_applied'],correction_xy_yaw=first['correction_xy_yaw'],pose=first['current_base_pose']),
   second=dict(correction_applied=second['correction_applied'],pose=second['current_base_pose']),actual_front=front,current_target=current,
   target_reference=str(reference_path),capture=str(capture),before_state=before,after_state=runtime.state(),nodes=[n for n in runtime.node.get_node_names() if 'localiz' in n])
 except Exception as exc:report.update(status='probe_failed_no_robot_commands',reason=str(exc))
 finally:
  if runtime:runtime.shutdown()
  rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report))

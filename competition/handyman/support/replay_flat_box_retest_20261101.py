import sys,json,cv2,numpy as np
from pathlib import Path
sys.path[:0]=['/__HANDYMAN_PACKAGE__/runtime','/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1']
from flat_box_geometry import operator_confirmed_source
from table_approach_reachability import base_path,plan_placement
from flat_box_transport import from_grasp,extents
from horizontal_can_grasp_plan import forward
from inverse_table_place_plan import FLEX,WRIST
from can_destination_surface import measure
b=Path('/__HANDYMAN_PACKAGE__/runtime');cfg=json.loads(Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());table=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']=='square_low_table#living_room')
p=b/'cigarette-grasp-20261101-03/captures/source';s=json.loads((p/'snapshot.json').read_text());d=np.load(p/'depth_m.npy');rgb=cv2.cvtColor(cv2.imread(str(p/'rgb.png')),cv2.COLOR_BGR2RGB)
ref=json.loads((b/'cigarette-grasp-20261101-01/held-context.json').read_text());m=operator_confirmed_source(rgb,d,s,table,ref['appearance'],ref['model'])
report=dict(actual_retest_frame_model=m,previous_command_reachability=base_path([2.0976,2.5751,1.6497],[1.09138,.03385,0.],table))
p=b/'cigarette-place-20261101-01/captures/001-table-before-extension';s=json.loads((p/'snapshot.json').read_text());d=np.load(p/'depth_m.npy');plane=measure(d,s,table)
held,_,_=from_grasp(b/'cigarette-grasp-20261101-01');_,r=forward([.25,FLEX,0,WRIST,0,held['holding_opening_rad']]);offset=r@held['center_palm_m'];v,h=extents(r@np.asarray(held['axes_palm']),held['sizes_m'])
hh=dict(center_minus_palm_m=offset.tolist(),radius_m=h,holding_opening_rad=held['holding_opening_rad'],bottom_below_palm_m=float(v-offset[2]))
try:
 plan=plan_placement(d,s,table,plane,hh,cfg,'offline-replay');report['original_frame_replanned']={k:v for k,v in plan.items() if k!='observed_table_support'}
except ValueError as e:report['original_frame_replan_reason']=str(e)
(b/'cigarette-reachability-offline-audit-20261101.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))

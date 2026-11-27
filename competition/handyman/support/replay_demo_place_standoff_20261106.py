"""Recompute stopped placement geometry from recorded frames; no ROS imports."""
import json,sys
from pathlib import Path
import numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');sys.path.extend([str(b),'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'])
from can_destination_surface import measure
from source_table_front import observed_front
from table_approach_reachability import plan_placement
from horizontal_can_grasp_plan import forward
from inverse_table_place_plan import FLEX,WRIST
from flat_box_transport import extents
from pregrasp_geometry import expected_horizontal_center
from apple_transfer_auto_plan import map_point
from pepper_place import extension_clearance
root=b/'cigarette-place-20261101-08'
cfg=json.loads((root/'config.snapshot.json').read_text());held=json.loads((root/'held-context.json').read_text());policy=json.loads((root/'outcome-policy.json').read_text())
destination=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==cfg['destination_id'])
_,r=forward([.25,FLEX,0,WRIST,0,held['holding_opening_rad']]);offset=r@held['center_palm_m'];vertical,horizontal=extents(r@np.asarray(held['axes_palm']),held['sizes_m'])
shape=dict(center_minus_palm_m=offset.tolist(),radius_m=horizontal,holding_opening_rad=held['holding_opening_rad'],bottom_below_palm_m=float(vertical-offset[2]))
rows=[]
for capture in sorted((root/'captures').iterdir()):
    snapshot=json.loads((capture/'snapshot.json').read_text());depth=np.load(capture/'depth_m.npy');volume=json.loads((capture/'object-measurement.json').read_text())
    plane=measure(depth,snapshot,destination)
    try:table,proof=observed_front(depth,snapshot,destination,plane)
    except ValueError as exc:table=destination;proof=dict(reason=str(exc))
    plan=plan_placement(depth,snapshot,table,plane,shape,cfg,'recorded-replay',held_volume=volume,outcome_policy=policy)
    predicted=map_point(snapshot,expected_horizontal_center(plan['palm_height_m'],offset))
    clearance=extension_clearance(predicted,snapshot,plane,plan)
    rows.append(dict(capture=str(capture),clearance=clearance,predicted=predicted.tolist(),table_front=proof,observed_xy_min=plane['observed_xy_min'],observed_xy_max=plane['observed_xy_max'],footprint_center=plan['footprint_center'],footprint_half_size=plan['footprint_half_size']))
out=b/'demo-place-standoff-replay-20261106.json';out.write_text(json.dumps(rows,indent=2));print(json.dumps(rows))

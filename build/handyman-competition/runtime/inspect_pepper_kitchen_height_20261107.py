"""Replay real recorded tabletop geometry; no ROS/control initialization."""
import json,sys
from pathlib import Path
import numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
root=Path(json.loads((b/'pepper-kitchen-live-latest.json').read_text())['output']);capture=root/'place/captures/table-before-extension'
snapshot=json.loads((capture/'snapshot.json').read_text());depth=np.load(capture/'depth_m.npy');held=json.loads((root/'held-context.json').read_text());cfg=json.loads((root/'config.snapshot.json').read_text())
dest=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==cfg['destination_id'])
from can_destination_surface import measure
from inverse_table_place_plan import OFFSET
plane=measure(depth,snapshot,dest);m=held['last_horizontal_measurement'];bottom=float(m['bottom_vertical_extent_m']-held['body_minus_palm_horizontal_m'][2]);palm=plane['table_height_map_m']+bottom+cfg['policy']['release_gap_m'];lift=palm-OFFSET
report=dict(table_height_m=plane['table_height_map_m'],palm_height_m=palm,required_lift_m=lift,physical_lift_max_m=.69,cleanup_lift_m=lift+.08,physical_range_sufficient=0<lift<.61,plane=plane,robot_commands_sent=False)
(root/'high-table-height-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))

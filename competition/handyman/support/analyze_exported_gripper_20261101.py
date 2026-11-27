import argparse,sys,json,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from gripper_physical_envelope import PhysicalHand
from horizontal_can_grasp_plan import forward
from thin_object_table_safety import plan

p=argparse.ArgumentParser();p.add_argument('--geometry',type=Path,required=True);p.add_argument('--prefab',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--height',type=float,required=True);args=p.parse_args()
report=json.loads(args.geometry.read_text())
hand=PhysicalHand(report,attached_object='cigarette#1');rows=[]
for label,q in [('horizontal',[.25,-1.7,0,.1292036732051034,0,.65]),('top_down',[.25,-math.pi/2,0,-math.pi/2,0,.65])]:
    _,r=forward(q);env=hand.envelope(args.prefab,contact_opening=hand.reference_opening)
    result=plan(dict(center_m=[0.,0.,args.height/2],height_m=args.height),[0,0,0],[0,0],r,env)
    if result['feasible']:result['nominal_palm_above_table_m']=(result['minimum_allowed_palm_height_m']+result['maximum_palm_height_for_contact_m'])/2
    rows.append(dict(posture=label,**result))
out=dict(physical_colliders=len(hand.parts),excluded_attached_object=hand.excluded,
    reference_opening_rad=hand.reference_opening,reference_transform_max_error=hand.reference_error,
    interval_motion_bound_m=env['interval_motion_bound_m'],plane_unbounded_diagnostic=True,postures=rows,robot_commands_sent=False)
with args.output.open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out))

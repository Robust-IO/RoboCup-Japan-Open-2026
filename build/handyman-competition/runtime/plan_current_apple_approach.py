"""Session-specific reviewed ROI evidence and conservative approach proposal.

No motion authorization. Visible surface is not a grasp center. This proposal
requires a new path screen and live gates, not automatic execution.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from compare_apple_alignment import surface


def main():
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    apple=surface(a.capture,[275,190,330,250],True)
    table=surface(a.capture,[350,255,470,295])
    s=json.loads((a.capture/'snapshot.json').read_text());j=s['joints']
    if abs(j['head_tilt_joint']+.5)>.005 or abs(j['head_pan_joint'])>.005:raise ValueError('ROI requires reviewed head pose')
    target=np.array(apple['median_base']);high=np.array([.689295215,.078,.72])
    if not (.75<target[0]<1.05 and -.1<target[1]<.2 and .50<target[2]<.65):raise ValueError('Unexpected target surface')
    # First approach budget only; leave at least 10cm forward gap at hypothetical high hand pose.
    dx=min(.09,max(0.,float(target[0]-high[0]-.10)))
    dy=float(np.clip(target[1]-high[1],-.04,.04))
    result=dict(motion_authorized=False,capture=str(a.capture),apple_visible_surface=apple,tabletop_patch=table,
                current_joints=j,proposed_total_base_translation_m=[dx,dy],
                hypothetical_high_finger_entrance_m=high.tolist(),
                residual_surface_minus_high_entrance_after_translation_m=(target-high-[dx,dy,0]).tolist(),
                max_single_segment_m=.015,
                limitations=['Manual RGB ROI and red pixels are diagnostic only, not a production object detector.',
                             'Tabletop patch provides height, not table front edge or free space.',
                             'High pose has not been executed or screened at the proposed endpoint.',
                             'No goal-center estimate or grasp closure authorized.',
                             'Current gripper is closed and head tilt is -0.5; do not invoke old tucked-open/-0.8 executor unchanged.'])
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='current_joints'},indent=2))


if __name__=='__main__':main()

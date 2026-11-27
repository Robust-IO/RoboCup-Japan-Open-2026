"""Generate a non-executable preapproach proposal from a reviewed visible patch."""
import argparse
import json
from pathlib import Path
import numpy as np
from compare_apple_alignment import surface
from hand_camera_ray import rays


def main():
    p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    s=json.loads((a.capture/'snapshot.json').read_text())
    patch=surface(a.capture,[301,302,337,334],True)
    observed=np.array(patch['median_base'])
    # Fixed bbox is specific to this reviewed capture, NOT a live detector API.
    ray=rays(s,[142.25453186,251.82557678,201.13162231,298.86068726],'sigverse_hsr_hand_v2')['base_ray']
    origin=np.array(ray['origin']);direction=np.array(ray['unit_direction'])
    hand_xy_at_patch_height=origin+direction*((observed[2]-origin[2])/direction[2])
    contact=np.array([.627169345+.06212587,.078,.72])
    delta=observed-contact
    # A 10 cm gap to the observed surface is deliberately left unclosed.
    # This proposal does not establish base/arm swept clearance or object center.
    advance=max(0.,float(delta[0]-.10))
    result=dict(motion_authorized=False,status='planning_only_requires_swept_clearance',
        source=str(a.capture),patch=patch,hand_ray_point_at_patch_height=hand_xy_at_patch_height.tolist(),
        preliminary_surface_minus_contact_m=delta.tolist(),
        proposed_base_translation_m=[advance,float(delta[1]),0.],
        proposed_palm_height_m=.72,remaining_surface_gap_m=.10,
        sequence=['Validate base, full extended arm and environment swept clearance.',
                  'Only then execute a bounded preapproach with feedback.',
                  'Stop and reacquire target; do not automatically descend or close.'],
        blockers=['Visible patch is not object center; partially occluded target.',
                  'Extended arm not covered by ordinary Nav2 footprint.',
                  'Map/laser swept clearance for translation has not been checked.',
                  'This file is not accepted by a motion executor.'])
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

"""Manually reviewed apple ROI diagnostic, not a production detector or grasp center."""
import argparse
import json
from pathlib import Path
import numpy as np
from compare_apple_alignment import surface
from hand_camera_ray import rotation


def main():
    p=argparse.ArgumentParser();p.add_argument('before',type=Path);p.add_argument('after',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    before=surface(a.before,[285,20,345,95],True)
    after=surface(a.after,[285,20,345,95],True)
    s=json.loads((a.after/'snapshot.json').read_text())
    def tf(d):
        m=np.eye(4);m[:3,:3]=rotation(d['quaternion']);m[:3,3]=d['translation'];return m
    palm=tf(s['base_from_hand'])@np.linalg.inv(tf(s['palm_from_hand']))
    # From reviewed finger geometry: entrance lies 62.12587mm along palm local Z.
    entrance=(palm@np.array([0.,0.,.06212587,1.]))[:3]
    target=np.array(after['median_base'])
    # Hypothetical earlier high pose, NOT a command from the current base pose.
    high_entrance=np.array([.627169345+.06212587,.078,.72])
    result=dict(motion_authorized=False,roi_source='Manually reviewed red apple only; not model detection',
        before=before,after=after,observed_surface_change_base_m=(target-np.array(before['median_base'])).tolist(),
        current_palm_base=palm[:3,3].tolist(),current_finger_entrance_base=entrance.tolist(),
        surface_minus_current_entrance_m=(target-entrance).tolist(),
        hypothetical_high_entrance_base=high_entrance.tolist(),
        surface_minus_hypothetical_high_entrance_m=(target-high_entrance).tolist(),
        limits=['Visible red surface median is NOT apple center or final grasp pose.',
                'Cross-frame base axes assumed nearly unchanged; difference is a consistency check, not odometry.',
                'Hypothetical arm extension must be re-screened at the new base pose before execution.',
                'Single-frame depth evidence is not grasp confirmation.'])
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

"""Historical RGBD replay only. Does not initialize ROS or command a robot."""
import argparse,json,math,time
from pathlib import Path
import cv2,numpy as np
from outcome_transfer_policy import load,hold_assessment
from flat_box_geometry import select_source,measure_near,operator_confirmed_source
from flat_box_partial_top import observe_known
from flat_box_transport import from_grasp,shape,extents
from canned_grasp_geometry import transform_point
from hand_camera_ray import rotation
from can_destination_surface import measure as measure_table
from table_approach_reachability import plan_placement
from flat_box_place_geometry import verify_placed
from horizontal_can_grasp_plan import forward
from inverse_table_place_plan import FLEX,WRIST
from apple_transfer_auto_plan import map_point
from thin_box_clearance import ThinBoxClearance,plane_in_map

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');policy=load()
    def capture(path):
        return dict(rgb=cv2.cvtColor(cv2.imread(str(path/'rgb.png')),cv2.COLOR_BGR2RGB),hand=cv2.imread(str(path/'hand.png')),
            depth=np.load(path/'depth_m.npy'),snapshot=json.loads((path/'snapshot.json').read_text()))
    cfg=json.loads((b/'cigarette-carry-20261101-03/config.snapshot.json').read_text())
    destinations=json.loads(Path(cfg['semantic_map']).read_text())['destinations']
    table=next(d for d in destinations if d['id']=='white_side_table#bedroom_1')
    source_root=b/'cigarette-grasp-20261101-08';source=capture(source_root/'captures/source')
    detections=json.loads((source_root/'captures/source/detections.json').read_text())['detections']
    selected,selection=select_source(source['rgb'],source['depth'],source['snapshot'],detections,table)
    model=json.loads((source_root/'source-geometry.json').read_text());appearance=json.loads((source_root/'appearance.json').read_text())
    contact=b/'cigarette-contact-20261101-02';sample=capture(contact/'captures/contact-002');s=sample['snapshot']
    old=json.loads((contact/'captures/contact-002/object-measurement.json').read_text())
    ro=rotation(source['snapshot']['tracking_transforms']['odom_from_base_at_depth']['quaternion'])
    axis=rotation(s['tracking_transforms']['odom_from_base_at_depth']['quaternion']).T@ro@model['axis_base']
    automatic=observe_known(sample['rgb'],sample['depth'],s,old['center_m'],axis,model,appearance,allow_locked=True)
    if not automatic.get('current_visible_surface_depth_measured',automatic.get('top_points',0)>0):raise ValueError('occlusion_replay_has_no_actual_depth')
    safety=ThinBoxClearance(b/'hsr-gripper-colliders-20261101-171937.json')
    _,r=forward([s['joints'][n] for n in ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')])
    # FK orientation plus exported image-time palm translation; no object truth.
    from scipy.spatial.transform import Rotation
    proof=safety.check(s,dict(translation=s['palm_base_at_depth'],quaternion=Rotation.from_matrix(r).as_quat().tolist()),
        automatic['center_m'],model['height_m'],plane_in_map(source['snapshot'],model['support_plane_base']))
    if proof['actual_palm_height_map_m']>proof['maximum_palm_height_for_contact_m']:raise ValueError('contact_height_replay_failed')
    result=json.loads((contact/'result.json').read_text());lifts=result['lift_evidence'][:policy['verification']['lift_frames']]
    if any(x['observed_lift_m']<policy['verification']['minimum_lift_m'] or x['follow_error_m']>policy['verification']['follow_error_m'] for x in lifts):raise ValueError('historical_actual_lift_not_accepted')
    held,prior,reference=from_grasp(contact)
    carry=b/'cigarette-carry-20261101-03';hand_checks=[]
    for name in ('after-tuck','destination-held'):
        current=capture(carry/'captures'/name)
        a=hold_assessment(reference,current['hand'],appearance)
        if a['state']!='consistent':raise ValueError('historical_hold_not_consistent:'+name)
        hand_checks.append(dict(capture=name,assessment=a))
    place=b/'cigarette-place-20261101-05';destination=next(d for d in destinations if d['id']=='square_low_table#living_room')
    before=next((place/'captures').glob('*table-after-standoff'));sample=capture(before);s=sample['snapshot']
    plane=measure_table(sample['depth'],s,destination);volume=shape(s,held);_,r=forward([.25,FLEX,0,WRIST,0,held['holding_opening_rad']])
    offset=r@held['center_palm_m'];v,h=extents(r@np.asarray(held['axes_palm']),held['sizes_m'])
    measured=dict(center_minus_palm_m=offset.tolist(),radius_m=h,holding_opening_rad=held['holding_opening_rad'],bottom_below_palm_m=float(v-offset[2]))
    plan=plan_placement(sample['depth'],s,destination,plane,measured,cfg,'historical-outcome-policy-replay',held_volume=volume,outcome_policy=policy)
    plan.update(class_name='cigarette',shape='flat_cuboid',height_m=model['height_m'],outcome_profile='outcome_v1')
    preexisting=operator_confirmed_source(sample['rgb'],sample['depth'],s,destination,appearance,model,operator_confirmed=False,return_all=True)
    old_plan=json.loads((place/'placement-plan.json').read_text());rows=[]
    for index in range(policy['verification']['placed_frames']):
        sample=capture(place/'captures'/f'placed-{index}');s=sample['snapshot']
        old_row=json.loads((place/'result.json').read_text())['placed_frames'][index]
        m=measure_near(sample['rgb'],sample['depth'],s,old_row['measurement']['center_m'],model,appearance)
        rows.append(dict(snapshot=s,measurement=m,plane=measure_table(sample['depth'],s,destination)))
    accepted=verify_placed(rows,plan,[0,0,0,-math.pi/2,0,.65],cleanup_completed=True)
    report=dict(status='historical_replay_passed',robot_commands_sent=False,live_validation=False,created_ns=time.time_ns(),policy=policy,
        source_selection=selection,source_confidence=selected['confidence'],integrated_occlusion=automatic,
        physical_contact_clearance=proof,actual_historical_lift_frames=len(lifts),hand_checks=hand_checks,
        placement_plan=dict(reachable=plan['base_reachability']['ready'],approach_distance_m=plan['planned_approach_distance_m'],
            previous_approach_distance_m=old_plan['planned_approach_distance_m'],edge_policy=plan['table_edge_policy'],patch=plan['patch']),
        preexisting_similar_boxes=len(preexisting),placed_verification=accepted,
        limitation='Historical frames do not prove the newly planned motion has been executed.')
    with args.output.open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(dict(status=report['status'],robot_commands_sent=False,output=str(args.output),placement=report['placement_plan'])))
if __name__=='__main__':main()

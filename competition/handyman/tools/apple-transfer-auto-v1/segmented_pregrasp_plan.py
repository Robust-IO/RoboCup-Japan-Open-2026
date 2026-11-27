"""Review-only segmented high-pose plan. No ROS imports or publishers."""
import json
import math

NAMES = ['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']


def build_plan():
    initial = [0.,0.,0.,-math.pi/2,0.,0.]
    lift = initial.copy(); lift[0] = .4428021848314039
    opened = lift.copy(); opened[5] = .65
    wrist = opened.copy(); wrist[3] = .1292036732051034
    extended = wrist.copy(); extended[1] = -1.7
    poses = [initial,lift,opened,wrist,extended]
    labels = ['lift','open','wrist_first','flex_second']
    stages = []
    for label,before,after in zip(labels,poses,poses[1:]):
        changed = [i for i,(a,b) in enumerate(zip(before,after)) if a != b]
        if len(changed) != 1:raise ValueError('One joint per stage required')
        stages.append(dict(name=label,expected_before=before,target=after,changed_joint=NAMES[changed[0]],
                           requires_fresh_feedback=True,requires_previous_stage_confirmed=True))
    return dict(motion_authorized=False,review_only=True,joint_names=NAMES,stages=stages,
                no_base_motion=True,no_descent=True,no_grasp_closure=True,
                unresolved=['Full head/torso/base self-collision coverage.',
                            'Fresh scene and exact base pose verification before execution.'])


def build_retract_plan():
    forward=build_plan()['stages']
    initial=forward[-1]['target'].copy()
    flex=initial.copy();flex[1]=0.
    wrist=flex.copy();wrist[3]=-math.pi/2
    lowered=wrist.copy();lowered[0]=0.
    return [dict(name=name,expected_before=before,target=after,changed_joint=joint)
            for name,before,after,joint in zip(
                ['retract_flex','restore_wrist','lower_tucked'],
                [initial,flex,wrist],[flex,wrist,lowered],
                ['arm_flex_joint','wrist_flex_joint','arm_lift_joint'])]


if __name__=='__main__':print(json.dumps(build_plan(),indent=2))

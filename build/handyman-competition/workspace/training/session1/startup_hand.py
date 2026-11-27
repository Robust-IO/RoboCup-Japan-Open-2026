"""Fresh-round empty-hand preparation using the task's existing runtime."""
import math

def prepare(runtime):
    runtime.sample(profile='state');state=runtime.state();joints=state['joints']
    expected=[0.,0.,0.,-math.pi/2,0.]
    if len(joints)<6 or not all(math.isfinite(x) for x in joints[:6]):raise ValueError('invalid_initial_joint_feedback')
    if any(abs(x-y)>.05 for x,y in zip(joints[:5],expected)):raise ValueError('not_initial_tucked_posture')
    if abs(joints[5]-.65)<=.02:
        return dict(status='empty_hand_already_open',motion_sent=False,base_motion_sent=False)
    goal=list(joints);goal[5]=.65
    result=runtime.move_joints(goal,'prepare_initial_empty_gripper',hand=True)
    return dict(status='empty_hand_opened',motion_sent=True,result=result,base_motion_sent=False)

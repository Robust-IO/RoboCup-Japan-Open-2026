"""Face the final approach route while folded, before extending over the bin."""
import math
import numpy as np
from task_recovery import check_cancel,motion_cancel

def face_container_route(stages,opening):
    rt=stages.runtime;motion=stages.motion
    # A guarded turn may first translate; recompute the bearing after that move.
    while True:
        check_cancel(motion_cancel(motion));rt.sample(profile='state');state=rt.state()
        pose=state['poses']['map'];center=np.asarray(opening['polygon_map_m'],float).mean(0)
        delta=center-np.asarray(pose[:2])
        if np.linalg.norm(delta)<.01:return
        heading=math.atan2(delta[1],delta[0]);error=math.atan2(math.sin(heading-pose[2]),math.cos(heading-pose[2]))
        stages.grasp_backend.record(dict(event='container_face_approach_route',pose=pose,target_heading_rad=heading,yaw_error_rad=error))
        if abs(error)<=.08:return
        if abs(error)>.08:
            motion.stage='approach_placement'
            motion.adjust('container-face-approach-route',pose,[0.,0.,error],state['joints'][:5],state['joints'][5])
        check_cancel(motion_cancel(motion));rt.sample(profile='state')

"""Use current support observations and existing finite adjustment executor."""
import numpy as np
from itertools import count
from task_recovery import check_cancel,motion_cancel,retry_read

class PlacementApproach:
    def __init__(self,observer,motion_backend,*,policy=None,maximum_corrections=5):
        if policy is None:import task_outcome_policy as policy
        self.observer=observer;self.backend=motion_backend;self.policy=policy
        if maximum_corrections!=5:raise ValueError('session1_uses_existing_five_corrections')
        self.maximum_corrections=maximum_corrections

    def __call__(self,plan):
        runtime=self.observer.runtime
        # Re-measure the plan every correction cycle; each
        # translation uses the entire measured delta, never 1.5 cm increments.
        for index in count():
            check_cancel(motion_cancel(self.backend))
            check=retry_read(lambda:self.observer.verify_support(plan),cancel=motion_cancel(self.backend))
            if check['ready']:return check
            if index and index%(self.maximum_corrections+1)==0:
                plan.update(self.observer.measure_and_plan())
                continue
            state=runtime.state();gap=check['bottom_gap_m']
            if not self.policy.RELEASE_BOTTOM_GAP_M[0]<=gap<=self.policy.RELEASE_BOTTOM_GAP_M[1]:
                goal=list(state['joints']);goal[0]+=self.observer.config['policy']['release_gap_m']-gap
                if not 0<=goal[0]<=.69:raise ValueError('placement_height_outside_lift_range')
                runtime.move_joints(goal,'correct_measured_release_height')
                continue
            snapshot=self.observer.last_row['item']['snapshot']
            r=self.observer.rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
            delta=r.T@np.r_[np.asarray(plan['release_xy_map_m'])-check['center_map_m'][:2],0.]
            if not np.isfinite(delta).all() or np.linalg.norm(delta[:2])<.001:
                raise ValueError('support_not_ready_without_valid_translation')
            self.backend.stage='approach_placement'
            self.backend.adjust(f'session1-place-approach-{index}',state['poses']['map'],
                                [float(delta[0]),float(delta[1]),0.],state['joints'][:5],
                                state['joints'][5],single_placement_approach=True)

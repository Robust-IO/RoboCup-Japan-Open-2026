"""Unlimited task-scoped recovery: four wrist attempts then new observation."""
from dataclasses import replace
import time
from .coordinator import AttemptFailure
from .grip_confirmation import accepted
from .recovery import Failure
from .planner import plan
from .templates import select_template
from .strategy_retry import too_wide_for_side

class CyclicCoordinator:
    def __init__(self,backend,hand,policy,record):
        self.b,self.hand,self.policy,self.record=backend,hand,policy,record
    def run(self,target,initial_plan=None):
        b=self.b;cycle=0;corrections=0;side_failed=False;needs_reset=False
        try:
            if b.holding_state()!='empty':return dict(status='initial_hand_not_empty',holding=b.holding_state())
            while True:
                b.check_task_cancelled()
                b.grasp_attempt_cycle=cycle+1
                if needs_reset:
                    b.reset_rotation_cycle();needs_reset=False
                    if hasattr(b,'prepare_grasp_staging'):
                        b.observe_stopped(target)
                        b.prepare_grasp_staging()
                cycle+=1
                self.record(dict(event='rotation_cycle_started',cycle=cycle,rotation_attempts=4,total_limit=None))
                candidate=None;decision={};approach_filtered=False
                try:
                    if initial_plan is not None:
                        scene,decision=initial_plan;initial_plan=None
                        self.policy=replace(self.policy,rotation_search=False,neutral_wrist_only=True)
                        b.policy=self.policy
                        decision=plan(scene,self.hand,self.policy)
                    else:
                        scene=getattr(b,'pending_grasp_scene',None)
                        b.pending_grasp_scene=None
                        if scene is None:scene=b.observe_stopped(target)
                        else:self.record(dict(event='staging_observation_handed_to_grasp',fresh_preextension_check_required=True))
                        planning_started=time.monotonic()
                        self.policy,selection=select_template(target,scene,self.policy)
                        self.policy=replace(self.policy,rotation_search=cycle>1,neutral_wrist_only=cycle==1,max_candidates=48,feasible_budget=8,lock_selected_pose=True)
                        self.policy=replace(self.policy,preferred_strategy='side',allowed_strategies=('side',))
                        from .local_strategy_selection import choose
                        self.policy,decision,selection=choose(b,scene,self.hand,self.policy,selection,
                            allow_overhead=cycle>1,planner=plan)
                        approach_filtered=True
                        self.record(dict(event='grasp_strategy_planning_timing',elapsed_s=time.monotonic()-planning_started))
                        self.record(dict(event='grasp_template_selected',**selection))
                        failed=getattr(b,'resume_failed_pose',None)
                        if failed is not None:
                            decision=b.rank_rotation_recovery(decision,failed)
                            approach_filtered=False
                    if hasattr(b,'filter_approach_decision') and not approach_filtered:
                        filter_started=time.monotonic()
                        decision=b.filter_approach_decision(decision)
                        self.record(dict(event='grasp_approach_filter_timing',elapsed_s=time.monotonic()-filter_started))
                    elif approach_filtered:
                        self.record(dict(event='grasp_approach_filter_reused',basis='same_decision_already_filtered_by_strategy_selection'))
                    candidate=decision['selected'];self.record(dict(event='grasp_plan',decision=decision))
                    if candidate is None:raise AttemptFailure(Failure.NO_CANDIDATE)
                    b.execute_candidate(candidate,scene)
                    evidence=b.verify_lift()
                    if accepted(vars(evidence)):return dict(status='grasp_verified',candidate=b.candidate,corrections=corrections,cycles=cycle,lift_evidence=vars(evidence))
                    raise AttemptFailure(Failure.HOLD_UNCERTAIN)
                except AttemptFailure as exc:
                    self.record(dict(event='rotation_cycle_initial_failure',cycle=cycle,reason=str(exc)))
                    trial=getattr(b,'try_aligned_trial_before_rotation',None)
                    if candidate is not None and callable(trial) and str(exc)=='contact_band_unobserved':
                        try:
                            if trial(str(exc)):
                                evidence=b.verify_lift()
                                if accepted(vars(evidence)):
                                    return dict(status='grasp_verified',candidate=b.candidate,corrections=corrections,cycles=cycle,lift_evidence=vars(evidence))
                        except AttemptFailure as trial_exc:
                            self.record(dict(event='aligned_trial_before_rotation_failed',reason=str(trial_exc)))
                    side_failed=too_wide_for_side(decision,self.policy.side_attempt_max_width_m)
                    if candidate is None or (exc.failure==Failure.OCCLUDED and b.holding_state()=='empty'):
                        needs_reset=True;continue
                # A possible held object must be resolved before rotating/opening.
                if b.holding_state()!='empty':
                    evidence=b.verify_lift()
                    if accepted(vars(evidence)):return dict(status='grasp_verified',candidate=b.candidate,corrections=corrections,cycles=cycle,lift_evidence=vars(evidence))
                    b.return_uncertain_to_source();needs_reset=True;continue
                b.begin_rotation_cycle()
                for attempt in range(1,5):
                    b.check_task_cancelled();corrections+=1
                    self.record(dict(event='rotation_only_correction',cycle=cycle,attempt=attempt,total_corrections=corrections,total_limit=None))
                    try:
                        b.rotate_only_retry(attempt)
                        evidence=b.verify_lift()
                        if accepted(vars(evidence)):return dict(status='grasp_verified',candidate=b.candidate,corrections=corrections,cycles=cycle,lift_evidence=vars(evidence))
                        raise AttemptFailure(Failure.HOLD_UNCERTAIN)
                    except AttemptFailure as exc:
                        self.record(dict(event='rotation_only_failed',cycle=cycle,attempt=attempt,reason=str(exc)))
                        if exc.failure==Failure.OCCLUDED and b.holding_state()=='empty':
                            self.record(dict(event='rotation_recovery_needs_new_observation',cycle=cycle,attempt=attempt))
                            break
                        if b.holding_state()!='empty':
                            b.return_uncertain_to_source();break
                b.resume_failed_pose=b.candidate
                needs_reset=True
        finally:b.stop()

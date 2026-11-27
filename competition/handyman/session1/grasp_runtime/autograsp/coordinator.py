"""Name-independent planning/execution loop. Backend owns RobotLock and ROS.

Every backend observation must stop motion and acquire a new synchronized scene.
Only verify_lift may declare success. Candidate selection is never a success.
"""
from dataclasses import dataclass, replace
from typing import Protocol
from .planner import Hand, Policy, Scene, plan
from .recovery import Failure, Recovery
from .grip_confirmation import accepted


class AttemptFailure(Exception):
    def __init__(self, failure, detail=''):
        self.failure = Failure(failure)
        super().__init__(detail or self.failure.value)


@dataclass(frozen=True)
class LiftEvidence:
    verified: bool
    object_followed: bool
    grip_retained: bool
    observations: int
    basis: str = "visual_lift"
    lift_distance_m: float = 0.
    reclose_blocked: bool = False
    minimum_lift_m: float = .05


class Backend(Protocol):
    def holding_state(self) -> str: ...
    def observe_stopped(self, target: str) -> Scene: ...
    def execute_candidate(self, candidate: dict, scene: Scene) -> None: ...
    def verify_lift(self) -> LiftEvidence: ...
    def recover(self, action: dict) -> None: ...
    def stop(self) -> None: ...


class Coordinator:
    def __init__(self, backend: Backend, hand: Hand, policy=Policy(), record=None):
        self.backend, self.hand, self.policy = backend, hand, policy
        self.record = record or (lambda event: None)

    def run(self, target, initial_plan=None):
        if getattr(self.backend,'cyclic_rotation_recovery',False):
            from .cyclic_recovery import CyclicCoordinator
            return CyclicCoordinator(self.backend,self.hand,self.policy,self.record).run(target,initial_plan)
        recovery = Recovery(self.policy.max_corrections)
        last_stamp = 0
        candidate = None
        side_failed = False
        selection={};decision={}
        from .strategy_retry import too_wide_for_side
        failed_pose = getattr(self.backend,'resume_failed_pose',None)
        if hasattr(self.backend,'resume_failed_pose'):del self.backend.resume_failed_pose
        # A failed side pose does not prove that the object is too wide.
        try:
            # A saved checkpoint is not a current holding measurement.
            state = self.backend.holding_state()
            if state != 'empty':
                return dict(status='initial_hand_not_empty', holding=state)
            while True:
                candidate = None
                try:
                    if initial_plan is not None:
                        scene,decision=initial_plan
                        initial_plan=None
                    else:
                        scene = self.backend.observe_stopped(target)
                        if getattr(self.backend,'use_family_templates',False):
                            from .templates import select_template
                            self.policy, selection = select_template(target, scene, self.policy)
                            if getattr(self.backend,'near_grasp_pose_handoff',False):
                                import numpy as np
                                if np.linalg.norm(np.median(scene.target,axis=0)[:2])<=1.1:
                                    self.policy=replace(self.policy,lock_selected_pose=True)
                                    selection.update(tracking='current_measured_shape_then_robot_feedback',near_grasp_handoff=True)
                            self.backend.policy = self.policy
                            self.record(dict(event='grasp_template_selected', **selection))
                        if side_failed:
                            self.policy=replace(self.policy,preferred_strategy='overhead',allowed_strategies=('overhead',))
                            self.backend.policy=self.policy
                        if failed_pose is not None:
                            self.policy=replace(self.policy,rotation_search=True,max_candidates=max(48,self.policy.max_candidates))
                            self.backend.policy=self.policy
                        decision = plan(scene, self.hand, self.policy, recovery.excluded)
                    if scene.stamp_ns <= last_stamp:
                        raise AttemptFailure(Failure.SENSOR_UNAVAILABLE, 'observation_not_new')
                    last_stamp = scene.stamp_ns
                    if decision.get('selected') is not None:
                        selected=decision['selected']
                        ties=[c for c in decision.get('candidates',[]) if c['strategy']==selected['strategy'] and c['score']>=selected['score']-.02]
                        if ties:
                            decision['selected']=min(ties,key=lambda c:sum(v*v for v in c['grasp_point_base_m'][:2]))
                            decision['near_tie_selection']='shorter_reach_within_0.02_score'
                    if failed_pose is not None and hasattr(self.backend,'rank_rotation_recovery'):
                        decision=self.backend.rank_rotation_recovery(decision,failed_pose)
                    self.record(dict(event='grasp_plan', decision=decision))
                    candidate = decision['selected']
                    if candidate is None:
                        raise AttemptFailure(Failure.NO_CANDIDATE)
                    # This backend call must check real posture/swept approach;
                    # the planner does not validate a whole arm trajectory.
                    self.backend.execute_candidate(candidate, scene)
                    evidence = self.backend.verify_lift()
                    if not accepted(vars(evidence)):
                        raise AttemptFailure(Failure.HOLD_UNCERTAIN)
                    return dict(status='grasp_verified', candidate=candidate,
                                corrections=recovery.corrections,
                                lift_evidence=vars(evidence))
                except AttemptFailure as exc:
                    # Persist across fresh template selection for THIS object attempt.
                    # A stale camera frame alone is not evidence that side grasp failed.
                    if candidate and candidate['strategy']=='side' and exc.failure in (
                            Failure.EMPTY_CLOSE,Failure.SLIPPED,Failure.MISALIGNED,Failure.APPROACH_COLLISION):
                        side_failed=too_wide_for_side(decision,self.policy.side_attempt_max_width_m) or selection.get('overhead_after_fold',False)
                        self.record(dict(event='side_grasp_retry_strategy',reason=exc.failure.value,overhead_next=side_failed))
                    if candidate and exc.failure in (Failure.MISALIGNED,Failure.EMPTY_CLOSE,Failure.SLIPPED,Failure.APPROACH_COLLISION):
                        failed_pose=candidate
                        if hasattr(self.backend,'capture_recovery_hand_hint') and self.backend.holding_state()=='empty':
                            self.backend.capture_recovery_hand_hint()
                    while True:
                        state = self.backend.holding_state()
                        action = recovery.decide(exc.failure,
                            candidate['id'] if candidate else None, holding=state)
                        if (side_failed or (failed_pose is not None and hasattr(self.backend,'rank_rotation_recovery'))) and state=='empty' and action['action']=='reobserve_then_correct_pose':
                            action=dict(action,action='withdraw_empty_reobserve_replan',strategy_next='overhead' if side_failed else 'side')
                        self.record(dict(event='grasp_recovery', detail=str(exc), **action))
                        if action['action'] == 'stop_with_evidence':
                            return dict(status='corrections_exhausted', holding=state,
                                        history=recovery.history)
                        try:
                            self.backend.recover(action)
                            break
                        except AttemptFailure as recovery_failure:
                            # Recovery is part of the same bounded attempt; its
                            # failures must not escape as an unhandled stop.
                            exc = recovery_failure
                    # Never start a new approach while an object may be held.
                    if self.backend.holding_state() != 'empty':
                        evidence = self.backend.verify_lift()
                        if accepted(vars(evidence)):
                            return dict(status='grasp_verified', candidate=candidate,
                                        corrections=recovery.corrections,
                                        lift_evidence=vars(evidence))
                        if getattr(self.backend,'automatic_source_return',False) and recovery.corrections<self.policy.max_corrections:
                            self.backend.return_uncertain_to_source()
                            if candidate and candidate['strategy']=='side':side_failed=too_wide_for_side(decision,self.policy.side_attempt_max_width_m) or selection.get('overhead_after_fold',False)
                            if candidate:recovery.excluded.add(candidate['id'])
                            continue
                        return dict(status='holding_unresolved_preserved',history=recovery.history)
        finally:
            self.backend.stop()

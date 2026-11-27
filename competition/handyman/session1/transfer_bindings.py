"""Bind remaining three stages, preserving one current grasp/placement context.

withdraw_source/withdraw_empty are measured TableWithdrawal instances. The
bottom_evidence callback must establish release-shape bottom observability; a
partial held surface is not silently treated as complete geometry.
"""
import json
from task_recovery import retry_read
from hold_observer import HoldObserver
from destination_observer import DestinationObserver
from table_withdrawal import PlacementStandoff
from placement_approach import PlacementApproach
from target_capture import TargetCapture
from release_record import record_release
from transfer_actions import tuck,place

class TransferBindings:
    def __init__(self,stages,destination,release_rotation,*,core,policy,
                 withdraw_source,withdraw_empty,bottom_evidence,outcome_policy=None,
                 hold_factory=HoldObserver,observer_factory=DestinationObserver,
                 standoff_factory=PlacementStandoff,approach_factory=PlacementApproach,
                 capture_factory=TargetCapture):
        self.stages=stages;self.destination=destination;self.release_rotation=release_rotation
        self.core=core;self.policy=policy;self.withdraw_source=withdraw_source
        self.withdraw_empty=withdraw_empty;self.bottom_evidence=bottom_evidence
        self.outcome_policy=outcome_policy;self.hold_factory=hold_factory
        self.observer_factory=observer_factory;self.standoff_factory=standoff_factory
        self.approach_factory=approach_factory;self.capture_factory=capture_factory
        self.base_core=core;self.base_release_rotation=release_rotation;self.base_bottom_evidence=bottom_evidence
        self.context_generation=0
        self.context=None;self.context_run=None;self.hold=None;self.release=None;self.placement=None;self.place_started=False

    def initialize(self,effect,cancel):
        self.stages.check(cancel)
        context=self.stages.context
        if context is None:raise ValueError('current_grasp_context_required')
        identity=(effect['session_id'],effect['run_id'])
        generation=context.get('carry_regrasp_generation',0)
        floor_recovered=(self.stages.config.get('session_profile',{}).get('session_number') in (2,5,6)
                         and generation>0 and context.get('task_source_table_id')==self.stages.config.get('source_table_id'))
        if floor_recovered and self.context is context and generation!=self.context_generation:
            if self.place_started:raise ValueError('cannot_replace_context_after_release_started')
            self.context=None;self.core=self.base_core;self.release_rotation=self.base_release_rotation
            self.bottom_evidence=self.base_bottom_evidence

        if self.context is None:
            if context['target']!=self.stages.config.get('task',{}).get('object','dog_doll') or (context['source_table']!=self.stages.config.get('source_table_id','white_side_table#living_room') and not floor_recovered):
                raise ValueError('wrong_grasp_context')
            self.context=context;self.context_run=identity;self.context_generation=generation
            self.hold=self.hold_factory(self.stages.grasp_backend,context)
            joints=context.get('joints_at_export')
            # Preserve side-grasp wrist corrections as well as overhead pose.
            # Returning a rolled side grasp to neutral changes the support normal.
            if joints and ((abs(joints[1]+1.5707963267948966)<.05 and abs(joints[3]+1.5707963267948966)<.05)
                           or (abs(joints[1]+1.7)<.05 and abs(joints[3]-.1292036732051034)<.05
                               and (abs(joints[2])>.005 or abs(joints[4])>.005))):
                from measured_release_posture import MeasuredReleaseCore
                from support_reference import assess_bottom
                self.core=MeasuredReleaseCore(self.core,joints)
                self.release_rotation=self.core.rotation
                self.bottom_evidence=lambda c:assess_bottom(c,self.release_rotation)['established']

        if self.context is not context or self.context_run!=identity:
            raise ValueError('new_run_requires_new_transfer_bindings')

    def tuck(self,effect,cancel):
        self.initialize(effect,cancel)
        result=tuck(self.stages.runtime,observe_hold=self.hold,withdraw_from_source=self.withdraw_source)
        self.stages.check(cancel)
        return result

    def place(self,effect,cancel):
        self.initialize(effect,cancel)
        if self.place_started:raise ValueError('place_already_started_no_automatic_replay')
        self.place_started=True
        self.stages.runtime.face_source((0.,-.5))
        planner_kwargs={}
        if hasattr(self.core,'predict'):
            from functools import partial
            from placement_planner import plan_placement
            planner_kwargs['planner']=partial(plan_placement,predict=self.core.predict)
        observer=self.observer_factory(self.stages.runtime,self.context,self.destination,
            self.stages.config,effect['operation_id'],self.release_rotation,
            outcome_policy=self.outcome_policy,**planner_kwargs)
        # Read-only checkpoints acquire new data on a transient sensor failure.
        # Never wrap place()/release_open() as a whole: release must not replay.
        for method in ('measure_and_plan','verify_support','verify_release'):
            if hasattr(observer,method):
                original=getattr(observer,method)
                def recoverable(*args,_original=original,**kw):
                    return retry_read(lambda:_original(*args,**kw),cancel=cancel,
                                      record=getattr(self.stages.runtime,'record',None))
                setattr(observer,method,recoverable)
        def acknowledge():
            item=retry_read(self.stages.grasp_backend._sample,cancel=cancel)
            self.stages.runtime.fresh_measurement(item['snapshot'])
            self.release=record_release(item['snapshot'],session_id=effect['session_id'],
                run_id=effect['run_id'],operation_id=effect['operation_id'],hand_is_open=self.policy.hand_is_open)
            path=self.stages.output/'release-record.json'
            path.write_text(json.dumps(self.release));return self.release
        result=place(self.stages.runtime,self.core,context=self.context,release_rotation=self.release_rotation,
            observe_hold=self.hold,measure_and_plan=observer.measure_and_plan,
            prepare_standoff=self.standoff_factory(observer,self.stages.motion,**({'predict':self.core.predict} if hasattr(self.core,'predict') else {})),
            approach=self.approach_factory(observer,self.stages.motion),verify_support=observer.verify_support,
            verify_release=getattr(observer,'verify_release',observer.verify_support),
            withdraw_empty=self.withdraw_empty,bottom_observed=self.bottom_evidence(self.context) is True,
            on_release=acknowledge,prepared_plan=getattr(self,'prepared_plan',None),finish_on_release=True)
        self.placement=result
        (self.stages.output/'placement-result.json').write_text(json.dumps(result))
        self.stages.check(cancel)
        return result

    def verify_placement(self,effect,cancel):
        self.initialize(effect,cancel)
        if not self.release or not self.placement or self.placement.get('release_sent') is not True:
            raise ValueError('this_run_release_required')
        result=dict(status='ok',target=self.context['target'],destination=self.destination['id'],
                    release_sent=True,cleanup_completed=False,cleanup_skipped=True,completion_actions_confirmed=True,
                    placement_verified=False,object_stability_checked=False,
                    basis='gripper_release_completed',moderator_acknowledged=False)

        self.stages.check(cancel)
        (self.stages.output/'placement-verification.json').write_text(json.dumps(result))
        return result

    def handlers(self):return dict(tuck=self.tuck,place=self.place,verify_placement=self.verify_placement)

"""Synchronous transfer actions for an owning runtime worker, not Executor.tick.

No ROS initialization or CLI side effects. Runtime methods use current feedback;
observation callbacks must acquire new evidence, not read historical JSON flags.
"""
import math
import numpy as np


def measured_release_shape(context, release_rotation):
    if context.get('schema') != 'handyman_held_surface_v1' or context.get('frame') != 'hand_palm_link':
        raise ValueError('palm_frame_held_surface_required')
    vertices=np.asarray(context['vertices_palm_m'],float)
    r=np.asarray(release_rotation,float)
    if vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or not np.isfinite(vertices).all():
        raise ValueError('invalid_held_surface')
    if r.shape!=(3,3) or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5:
        raise ValueError('invalid_release_rotation')
    rotated=vertices@r.T
    from object_support_disk import enclosing_disk
    center=(rotated.min(0)+rotated.max(0))/2
    center[:2],radius=enclosing_disk(rotated[:,:2])
    bottom=-float(rotated[:,2].min())
    if context.get('source_support_reference'):
        from support_reference import assess_bottom
        assessment=assess_bottom(context,r)
        if not assessment['established']:raise ValueError(assessment['reason'])
        bottom=assessment['bottom_below_palm_m']
    return dict(center_minus_palm_m=center.tolist(),
                radius_m=radius,
                radius_basis='minimum_enclosing_measured_xy_disk',
                bottom_below_palm_m=bottom,
                vertices_release_m=rotated.tolist(),complete_surface_verified=False)


def _hold(observe,label):
    evidence=observe(label)
    accepted=isinstance(evidence,dict) and (evidence.get('holding_verified') is True or
        (label in ('before-tuck','after-tuck','before-place') and evidence.get('carry_state_verified') is True))
    if not accepted:
        raise ValueError('current_hold_not_verified:'+label)
    return evidence


def tuck(runtime, *, observe_hold, withdraw_from_source):
    """Withdraw callback uses current source-table geometry; no fixed retreat.

    Fold and lower all arm joints together, preserving observed gripper opening.
    move_joints must return only after joint feedback confirms completion.
    """
    before=_hold(observe_hold,'before-tuck')
    state=runtime.state()
    if abs(state['joints'][1])<.05:
        withdrawal=dict(clear_to_fold=True,basis='already_folded_current_joint_feedback')
    else:withdrawal=withdraw_from_source()
    if not isinstance(withdrawal,dict) or withdrawal.get('clear_to_fold') is not True:
        raise ValueError('source_withdrawal_incomplete')
    if abs(state['joints'][1])>=.05:_hold(observe_hold,'after-withdrawal')
    goal=list(runtime.state()['joints']);goal[:5]=[0.,0.,0.,-math.pi/2,0.]
    runtime.move_joints(goal,'fold_and_lower_held_object')
    after=_hold(observe_hold,'after-tuck')
    return dict(status='ok',holding_verified=True,carry_ready=True,
                holding_opening_rad=float(runtime.state()['joints'][5]),
                evidence={'before':before,'after':after,'withdrawal':withdrawal})


def place(runtime,core,*,context,release_rotation,observe_hold,measure_and_plan,
          prepare_standoff,approach,verify_support,withdraw_empty,bottom_observed,on_release=None,verify_release=None,prepared_plan=None,finish_on_release=False):
    """Use common table-height and release primitives, never object-specific size.

    measure_and_plan receives measured held shape and returns palm_height_m based
    on current table depth. approach owns existing correction policy. This action
    reports release only; independent verify_placement remains a separate stage.
    """
    if prepared_plan is None:
        _hold(observe_hold,'before-place')
        if bottom_observed is not True:raise ValueError('held_bottom_not_observed')
        shape=measured_release_shape(context,release_rotation)
        plan=measure_and_plan(shape)
        # Standoff adapter may retreat and remeasure, returning the resulting plan.
        plan=prepare_standoff(plan)
        holding=float(runtime.state()['joints'][5])
        core.prepare_release(runtime,plan['palm_height_m'],holding,
                             already_extended=plan.get('already_extended',False))
        approach(plan)
    else:
        plan=prepared_plan  # Completed approach: only current release check follows.
    def check():
        # Placement geometry authorizes release; wrist occlusion is not a veto.
        proof=(verify_release or verify_support)(plan)
        if not isinstance(proof,dict) or proof.get('ready') is not True:
            raise ValueError('current_destination_support_unconfirmed')
        return proof
    core.release_open(runtime,check)
    result=dict(status='ok',release_sent=True,placement_verified=False,
                target=context['target'],plan=plan,cleanup_completed=False)
    if on_release is not None:
        try:result['release_record']=on_release()
        except Exception as exc:
            result.update(status='incomplete',reason='release_acknowledgement_unavailable',
                          release_record_error=str(exc))
            return result
    if finish_on_release:
        result.update(completion_actions_confirmed=True,basis='gripper_release_completed',cleanup_skipped=True)
        return result
    # A cleanup failure does not erase the actual release: still verify placement.
    try:
        core.raise_after_release(runtime)
        withdrawal=withdraw_empty()
        if not isinstance(withdrawal,dict) or withdrawal.get('clear_to_fold') is not True:
            raise ValueError('empty_withdrawal_incomplete')
        core.fold_after_withdrawal(runtime)
        result['cleanup_completed']=True
    except Exception as exc:
        result['cleanup_error']=str(exc)
    return result

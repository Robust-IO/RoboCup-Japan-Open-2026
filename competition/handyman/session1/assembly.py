"""Assemble all eight handlers from live resources; no initialization on import.

Resources must be owned by the caller under RobotLock. request_stop must return
actual motion-stop confirmation; bottom_evidence must use measured geometry.
Neither requirement is replaced by a constant-success default.
"""
from stage_bindings import StageBindings
from transfer_bindings import TransferBindings
from table_withdrawal import TableWithdrawal
from withdrawal_observer import WithdrawalObserver
from ros_protocol import bind_application


def assemble(*,node,session_id,runtime,worker,grasp_backend,motion_backend,config,
             environment,semantic,output,bottom_evidence=None,request_stop=None,outcome_policy=None,container_adapter_factory=None):
    import table_transfer_core as core
    import task_outcome_policy as policy
    from grasp_runtime.autograsp.kinematics import forward
    controller=None
    if request_stop is None:
        from motion_stop import MotionGate,StopController
        gate=MotionGate();gate.install(runtime,motion_backend)
        controller=StopController(runtime,motion_backend,gate,output)
        request_stop=controller.request_stop
    if outcome_policy is None:
        from outcome_transfer_policy import load
        outcome_policy=load()
    from session_placement_policy import relaxed_support_policy
    outcome_policy=relaxed_support_policy(outcome_policy)
    tables={t['id']:t for t in semantic['destinations']}
    source=tables[config.get('source_table_id','white_side_table#living_room')]
    destination=tables[config.get('destination_id','square_low_table#living_room')]
    profile=config.get('session_profile')
    container=profile is not None and profile['destination']['mode']=='container_drop'
    if container and container_adapter_factory is None:
        raise ValueError('container_live_adapter_required_before_execution')
    if grasp_backend.table['id']!=source['id'] or grasp_backend.worker is not worker:
        raise ValueError('shared_grasp_source_and_worker_required')
    from candidate_selection import load_selector
    selector=load_selector(config,semantic)
    stages=StageBindings(runtime,motion_backend,grasp_backend,config,environment,output,candidate_selector=selector)
    from shared_turn_guard import install
    install(stages)
    source_geometry=WithdrawalObserver(grasp_backend,source,lambda:stages.context)
    empty_geometry=WithdrawalObserver(grasp_backend,destination,lambda:None)
    _,release_rotation=forward([.25,core.FLEX,0.,core.WRIST,0.,.3])
    if bottom_evidence is None:
        from support_reference import assess_bottom
        bottom_evidence=lambda context:assess_bottom(context,release_rotation)['established']
    transfer=TransferBindings(stages,destination,release_rotation,core=core,policy=policy,
        withdraw_source=TableWithdrawal(runtime,motion_backend,source_geometry,label='source-withdraw'),
        withdraw_empty=TableWithdrawal(runtime,motion_backend,empty_geometry,label='empty-withdraw'),
        bottom_evidence=bottom_evidence,outcome_policy=outcome_policy)
    if container:
        from container_bindings import ContainerBindings
        transfer=ContainerBindings(stages,transfer,container_adapter_factory)
    kwargs={'profile':profile} if profile is not None else {}
    app,protocol=bind_application(node,session_id,stages.handlers(transfer.handlers()),request_stop,**kwargs)
    if controller is not None:controller.bind(app)
    # Readiness is intentionally not set here: owner must verify current startup.
    return dict(application=app,protocol=protocol,stages=stages,transfer=transfer,stop_controller=controller)

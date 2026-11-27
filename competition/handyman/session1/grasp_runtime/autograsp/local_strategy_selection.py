"""A feasible local side grasp outranks global shape-ratio hints."""
from dataclasses import replace

def choose(backend,scene,hand,policy,selection,*,allow_overhead,planner):
    side=replace(policy,preferred_strategy='side',allowed_strategies=('side',))
    backend.policy=side
    decision=planner(scene,hand,side)
    if hasattr(backend,'filter_approach_decision'):
        decision=backend.filter_approach_decision(decision)
    if decision.get('selected') is not None or not allow_overhead:
        return side,decision,dict(selection,preferred_strategy='side',allowed_strategies=['side'],
            selection_reason='local_side_candidate_precedes_global_ratio' if decision.get('selected') is not None else 'first_cycle_side_search')
    overhead=replace(policy,preferred_strategy='overhead',allowed_strategies=('overhead',))
    backend.policy=overhead
    top=planner(scene,hand,overhead)
    if hasattr(backend,'filter_approach_decision'):top=backend.filter_approach_decision(top)
    if top.get('selected') is not None:
        return overhead,top,dict(selection,preferred_strategy='overhead',allowed_strategies=['overhead'],selection_reason='no_executable_local_side_candidate_after_reobserve')
    # Never latch overhead-only when both plans fail. Next observation starts
    # by considering side grasp again.
    backend.policy=side
    decision=dict(decision,overhead_rejected=top.get('rejected',{}))
    return side,decision,dict(selection,preferred_strategy='side',allowed_strategies=['side'],selection_reason='both_strategies_require_new_observation')

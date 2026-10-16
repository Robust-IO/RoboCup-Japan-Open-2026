"""Project policy, not a claim about official competition rules.

Consumes structural analysis evidence from a future trusted map analyser.
Does not calculate reachability, send protocol messages, or control robots.
"""
from dataclasses import dataclass
import re

@dataclass(frozen=True)
class UnreachablePolicy:
    explicit_room_as_absent: bool = False

def decide_room(*, room, requested_room, map_sha256, evidence, policy=UnreachablePolicy()):
    result=dict(room=room,decision='unresolved',reason='insufficient_structural_evidence',
                protocol_response=None,actionable=False,rules_verified=False)
    # An exclusion, navigation timeout, or one failed candidate is not evidence.
    if not isinstance(evidence,dict): return result
    if not isinstance(map_sha256,str) or not re.fullmatch('[0-9a-f]{64}',map_sha256): return result
    required=dict(schema='handyman-structural-reachability-v1',room=room,map_sha256=map_sha256,
                  status='structurally_unreachable',method='static_configuration_space_connectivity',
                  all_room_access_candidates_checked=True,unknown_space_affects_result=False,
                  transient_obstacles_used=False,start_component_valid=True)
    if any(type(evidence.get(k)) is not type(v) or evidence.get(k)!=v for k,v in required.items()):
        return result
    if not evidence.get('analysis_id') or not evidence.get('robot_footprint_id'):
        return result
    result.update(reason='room_structurally_unreachable',analysis_id=evidence['analysis_id'])
    if requested_room!=room:
        result['decision']='skip_room'
    elif policy.explicit_room_as_absent:
        result.update(decision='project_policy_absent',protocol_response='Does_not_exist')
    else:
        result['decision']='task_incomplete'
    return result

def decide_absence(*, task_id, target, room, map_sha256, search_evidence=None,
                   structural_evidence=None, policy=UnreachablePolicy()):
    """Two explicit reasons only; callers must supply independently checked evidence."""
    structural=decide_room(room=room,requested_room=room,map_sha256=map_sha256,
                           evidence=structural_evidence,policy=policy)
    if not task_id or not target or not room:
        return dict(decision='unresolved',reason='missing_task_identity',protocol_response=None,actionable=False)
    if structural['protocol_response'] is not None:
        return dict(structural,task_id=task_id,target=target)
    result=dict(task_id=task_id,target=target,room=room,decision='unresolved',
                reason='incomplete_or_invalid_search',protocol_response=None,actionable=False,rules_verified=False)
    if not isinstance(search_evidence,dict): return result
    if not isinstance(map_sha256,str) or not re.fullmatch('[0-9a-f]{64}',map_sha256): return result
    required=dict(schema='handyman-room-search-evidence-v1',task_id=task_id,target=target,room=room,
                  map_sha256=map_sha256,room_reachable=True,coverage_verified=True,
                  perception_valid=True,target_found=False,search_complete=True)
    if any(type(search_evidence.get(k)) is not type(v) or search_evidence.get(k)!=v for k,v in required.items()):
        return result
    views=search_evidence.get('required_view_ids')
    done=search_evidence.get('completed_view_ids')
    if not isinstance(views,list) or not views or not all(isinstance(x,str) and x for x in views): return result
    if not isinstance(done,list) or not all(isinstance(x,str) and x for x in done): return result
    if len(set(views))!=len(views) or len(set(done))!=len(done) or set(views)!=set(done): return result
    if search_evidence.get('failed_view_ids')!=[] or search_evidence.get('skipped_view_ids')!=[]: return result
    if not search_evidence.get('coverage_analysis_id'): return result
    return dict(result,decision='searched_room_absent',reason='target_absent_after_verified_room_search',
                protocol_response='Does_not_exist')

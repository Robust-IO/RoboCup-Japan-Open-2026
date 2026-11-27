"""Offline scheduling from a freshly built connectivity-enriched plan."""
from search_scan import SearchScan
from static_room_policy import decide

def room_scan(plan,room,task_id,target,started=0.):
    report=plan.get('connectivity')
    if not isinstance(report,dict) or report.get('layout')!=plan.get('layout'):
        return None,'missing_or_mismatched_connectivity'
    decision=decide(report,room,plan.get('map_bundle_sha256'),enable_static_rule=True)
    if decision['decision']!='retain_as_reachable':
        return None,decision.get('reason','connectivity_review_required')
    matches=[r for r in plan.get('rooms',[]) if r.get('room')==room]
    if len(matches)!=1: return None,'missing_or_ambiguous_room'
    r=matches[0]
    if r.get('valid') is not True or r.get('errors') or not r.get('points'):
        return None,'invalid_room_plan'
    if plan.get('frame_id')!='map' or any(p.get('issues') or p.get('pose') is None for p in r['points']):
        return None,'invalid_point_plan'
    try: return SearchScan(task_id,target,[p['pose'] for p in r['points']],started),'eligible_for_offline_rehearsal'
    except (ValueError,KeyError,TypeError): return None,'invalid_point_plan'

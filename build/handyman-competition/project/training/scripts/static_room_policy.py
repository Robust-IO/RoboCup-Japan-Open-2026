"""User-selected rule: a connected static map is considered traversable.

Offline decisions only. Dynamic blockage and navigation failure never imply absence.
Input must come from the static-map analyser, not a live obstacle costmap.
"""
import re

def decide(report,room,current_map_sha256,*,explicit_room=False,enable_static_rule=False):
    out=dict(room=room,decision='review_required',protocol_response=None,actionable=False,
             assumption='connected_static_map_is_traversable',rules_verified=False)
    if not enable_static_rule: return dict(out,reason='static_rule_disabled')
    if not isinstance(current_map_sha256,str) or not re.fullmatch('[0-9a-f]{64}',current_map_sha256): return dict(out,reason='invalid_map_identity')
    if report.get('map_bundle_sha256')!=current_map_sha256: return dict(out,reason='stale_map_analysis')
    if report.get('method')!='optimistic_8_connected_point_agent' or report.get('initial_known_free') is not True:
        return dict(out,reason='invalid_analysis_or_start')
    matches=[r for r in report.get('rooms',[]) if r.get('room')==room]
    if len(matches)!=1: return dict(out,reason='missing_or_ambiguous_room')
    r=matches[0]; counts=r.get('cells',{})
    values=[counts.get(k) for k in ['known_free','known_connected','optimistic_traversable','optimistic_connected']]
    if any(type(v) is not int or v<0 for v in values): return dict(out,reason='invalid_cell_counts')
    free,connected,total,optimistic=values
    if not 0<=connected<=free<=total or not connected<=optimistic<=total:
        return dict(out,reason='inconsistent_cell_counts')
    if r.get('status')=='point_connectivity_present' and connected>0:
        return dict(out,decision='retain_as_reachable',reason='static_map_connected')
    if r.get('status')=='structural_disconnection_candidate' and free>0 and optimistic==0:
        return dict(out,decision='specified_room_absent' if explicit_room else 'skip_room',
                    protocol_response='Does_not_exist' if explicit_room else None,
                    reason='room_disconnected_in_static_map')
    return dict(out,reason='unknown_space_or_invalid_room')

def navigation_failure(*,guest_blocking=False):
    return dict(decision='wait_and_retry' if guest_blocking else 'diagnose_navigation',
                protocol_response=None,permanently_exclude_room=False,actionable=False)

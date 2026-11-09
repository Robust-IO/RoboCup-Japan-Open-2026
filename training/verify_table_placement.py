"""Verify a completed recorded tabletop release, not a competition success report."""
import argparse
import json
import math
from pathlib import Path
from tabletop_support import support_from_plan
from task_outcome_policy import (PLACED_BOTTOM_GAP_M, PLACED_CENTER_SPREAD_M,
                                 hand_is_open, robot_ready)


def verify(audit,prior,actions,destination='white_side_table#bedroom_1',room='bedroom',*,placement_plan=None):
    support = None
    if placement_plan is not None:
        if placement_plan.get('destination_id') != destination:
            raise ValueError('placement_edge_destination_mismatch')
        support = support_from_plan(placement_plan)
    if actions['release'].get('status')!='passed':
        raise ValueError('completed_release_required')
    if not any(e.get('event')=='command' and e.get('kind')=='hand' and abs(e['positions'][5]-.65)<.001 for e in actions['release'].get('events',[])):
        raise ValueError('release_command_not_recorded')
    rows=[];rejected=[]
    for frame in audit['frames']:
        candidates=[x for x in frame['apple_measurements'] if (support is not None or x['inside_table_with_apple_margin']) and math.dist(x['center_map'][:2],prior['center_map'][:2])<.10 and abs(x['estimated_bottom_gap_m'])<=PLACED_BOTTOM_GAP_M]
        joints=frame['joints']
        if len(candidates)!=1 or not hand_is_open(joints.get('hand_motor_joint')):
            rejected.append(frame['capture']);continue
        candidate=candidates[0]
        # With measured tabletop support, table ownership, height, continuity
        # and released hand already decide placement; camera standoff is
        # not an additional success criterion. Preserve legacy callers' gate.
        if support is None and candidate['candidate']['center_m'][0]<.55:rejected.append(frame['capture']);continue
        edge = None if support is None else support.query(candidate['center_map'][:2], candidate['candidate']['radius_m'])
        if edge is not None and not edge['safe']:
            rejected.append(frame['capture']);continue
        rows.append(dict(capture=frame['capture'],stamp_ns=frame['source_stamp_ns'],center_map=candidate['center_map'],bottom_gap_m=candidate['estimated_bottom_gap_m'],confidence=candidate['confidence'],footprint_margin_m=candidate['footprint_margin_m'],table_edge_safety=edge))
    span=(rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9 if len(rows)>1 else 0
    spread=max((math.dist(x['center_map'],rows[-1]['center_map']) for x in rows),default=None)
    stamps=[r['stamp_ns'] for r in rows]
    unique_ordered=(len({r['capture'] for r in rows})==len(rows)
                    and all(type(s) is int and s>0 for s in stamps)
                    and all(a<b for a,b in zip(stamps,stamps[1:])))
    passed=len(rows)>=3 and span>=1 and spread<=PLACED_CENTER_SPREAD_M and not rejected and unique_ordered
    ready=bool(audit['frames'] and robot_ready(audit['frames'][-1]['joints']))
    cleanup=all(actions.get(k,{}).get('status')=='passed' for k in ('raise','retract'))
    return dict(status='table_placement_verified' if passed else 'inconclusive',placement_success_verified=passed,table_edge_safety_verified=bool(passed and support is not None),object='apple',destination=destination,room=room,frames=len(rows),span_s=span,center_spread_m=spread,rows=rows,rejected=rejected,robot_tucked_and_gripper_open=ready,robot_ready_for_next_task=bool(ready and cleanup),cleanup_actions_completed=cleanup,cleanup_required=bool(not ready or not cleanup),protocol_task_finished_sent=False,motion_authorized=False,limitation='One simulation transfer trial; camera timing/absolute calibration uncertainty remains. Not a placement success-rate estimate. Legacy callers without a placement plan do not establish tabletop edge safety. Placement is decided by released object/support/stability; robot cleanup is reported separately.')


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--prefix',default='bedroom');p.add_argument('--destination-id',default='white_side_table#bedroom_1');p.add_argument('--room',default='bedroom');p.add_argument('--audit-file',type=Path);p.add_argument('--placement-plan',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('output_exists')
    read=lambda p:json.loads(p.read_text())
    actions={k:read(a.directory/f'{a.prefix}-place-{k}-01'/'result.json') for k in ('release','raise','retract')}
    result=verify(read(a.audit_file or a.directory/f'{a.prefix}-final-apple-audit.json'),read(a.directory/f'{a.prefix}-pre-release-apple-geometry.json'),actions,a.destination_id,a.room,placement_plan=read(a.placement_plan) if a.placement_plan else None)
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result))
    if not result['placement_success_verified']:raise SystemExit(1)


if __name__=='__main__':main()

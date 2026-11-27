"""Read successful transfer journals; do not import ROS or command the robot."""
import argparse
import json
from pathlib import Path


def summarize(root):
    root = Path(root)
    report = json.loads((root/'result.json').read_text())
    if report.get('physical_transfer_verified') is not True:
        raise ValueError('only_completed_physical_transfer_records_are_comparable')
    events = [json.loads(line) for line in (root/'journal.jsonl').read_text().splitlines()]
    stages = [row for row in events if row.get('event') == 'stage']
    spans = []
    totals = dict(source_navigation=0., source_vision=0., pickup=0.,
                  destination_navigation=0., placement=0., setup=0.)
    for i, row in enumerate(stages):
        finish = stages[i+1]['time_ns'] if i+1 < len(stages) else report['completed_ns']
        seconds = (finish-row['time_ns'])/1e9
        if seconds < 0:
            raise ValueError('wall_clock_jump_prevents_stage_timing')
        name = row['stage']
        if name == 'navigate_source': group = 'source_navigation'
        elif name.startswith('new_') and name.endswith('_rgbd'): group = 'source_vision'
        elif name == 'navigate_destination': group = 'destination_navigation'
        elif name in ('preflight', 'head_view'): group = 'setup'
        elif name in ('measure_destination_and_plan', 'prepare_horizontal_for_placement',
                      'approach_measured_release_region', 'release_once_from_current_support',
                      'withdraw_and_retract_open_hand', 'verify_actual_placement'): group = 'placement'
        else: group = 'pickup'
        totals[group] += seconds
        spans.append(dict(stage=name, seconds=round(seconds, 3), group=group))
    elapsed = report['elapsed_s']
    navigation = totals['source_navigation']+totals['destination_navigation']
    return dict(run=str(root), elapsed_s=elapsed, initial_pose=report.get('initial_base_map_pose'),
                seconds_by_group={k: round(v, 3) for k, v in totals.items()},
                navigation_share=navigation/elapsed, unclassified_setup_s=elapsed-sum(totals.values()),
                stages=spans, comparison_note='Starting poses differ; these are not a controlled speed-profile comparison.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = dict(robot_commands_sent=False, runs=[summarize(root) for root in args.runs])
    if args.output:
        with args.output.open('x') as stream:
            json.dump(result, stream, indent=2)
    print(json.dumps(result))


if __name__ == '__main__':
    main()

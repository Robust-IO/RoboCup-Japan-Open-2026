"""Offline Session1 event replay. Never imports ROS or starts robot programs."""
import argparse
import json
from pathlib import Path
from session1 import Session1


def replay(events, session_id, source_table=None):
    runner = Session1(session_id, source_table=source_table)
    timeline = []
    for number, event in enumerate(events, 1):
        kind = event['kind']
        if kind == 'preflight':
            runner.preflight(ready=event['ready'])
        elif kind == 'moderator':
            runner.moderator(event['message'], event.get('detail', ''),
                             session_id=event.get('session_id', session_id))
        elif kind == 'result':
            # Replay-local token resolution is only for explicit synthetic
            # fixtures. Live results must use exact emitted operation/run IDs.
            runner.result(runner.pending if event.get('operation_id') == '$pending' else event.get('operation_id'),
                          event['result'], session_id=event.get('session_id', session_id),
                          run_id=runner.run_id if event.get('run_id') == '$run' else event.get('run_id'))
        elif kind == 'cancel':
            runner.stop(event.get('reason', 'operator_cancel'), 'cancelled')
        else:
            raise ValueError(f'unknown_event_kind:{kind}')
        timeline.append(dict(line=number, state=runner.state, effects=runner.drain()))
    return dict(mode='offline_replay', physical_transfer_verified=False,
                robot_commands_sent=0, protocol_messages_sent=0,
                final=runner.snapshot(), timeline=timeline)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--events', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--session-id', default='session1-offline')
    parser.add_argument('--source-table', choices=['white_side_table#living_room'],
                        help='Known pickup furniture; skip room-wide search, retain current visual localization')
    args = parser.parse_args()
    events = [json.loads(line) for line in args.events.read_text().splitlines() if line.strip()]
    report = replay(events, args.session_id, args.source_table)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('mode','physical_transfer_verified','robot_commands_sent')}))
    print('state='+report['final']['state'])
    return 0 if report['final']['state'] == 'succeeded' else 2


if __name__ == '__main__':
    raise SystemExit(main())

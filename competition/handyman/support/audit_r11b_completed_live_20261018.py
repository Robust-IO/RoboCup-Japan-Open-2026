"""Read-only physical-result audit and new evidence archive; no ROS commands."""
import json
import math
from pathlib import Path
import tarfile

base = Path('/__HANDYMAN_PACKAGE__/runtime')
run = base / 'apple-r11b-reviewed1280-transfer-20261018-01'
out = base / 'apple-r11b-live-final-audit-20261018-01.json'
archive = base / 'apple-r11b-live-final-evidence-20261018-01.tar.gz'

def read(name):
    return json.loads((run / name).read_text())

result = read('result.json')
if result.get('status') != 'reviewed_r11b_grasp_carry_placement_verified':
    raise ValueError('completed_reviewed_trial_required')
verification = read('placement-verification.json')
sequence = read('placement-approach-sequence.json')
navigation = read('destination-navigation.json')
final = sorted((run / 'placed-final').glob('*/snapshot.json'))[-1].parent
snapshot = json.loads((final / 'snapshot.json').read_text())
motion_names = ('apple_transfer_auto.py', 'reviewed_r11b_1280_grasp_place_20261018.py',
                'run_apple_close_lift_trial.py', 'pregrasp_raised_trial.py',
                'reviewed_carry_adjustment.py', 'navigate_carry.py')
running = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        args = [a.decode(errors='replace') for a in (proc / 'cmdline').read_bytes().split(b'\0') if a]
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    scripts = [Path(a).name for a in args[1:] if a.endswith('.py')]
    if any(a in motion_names for a in scripts):
        running.append(dict(pid=int(proc.name), scripts=scripts))

summary = dict(
    status='completed_live_result_audited_no_robot_command',
    result=result,
    placement_verified=verification['placement_success_verified'],
    edge_safety_verified=verification['table_edge_safety_verified'],
    placement_approach_commands=len(sequence['steps']),
    placement_translation_m=sequence['total_translation_m'],
    placement_step_deltas=[s['delta_m'] for s in sequence['steps']],
    final_placement_span_s=verification['span_s'],
    final_center_spread_m=verification['center_spread_m'],
    minimum_observed_outer_clearance_estimate_m=min(
        r['table_edge_safety']['outer_clearance_lower_estimate_m'] for r in verification['rows']),
    final_capture=str(final),
    final_snapshot=snapshot,
    destination_navigation=navigation,
    matching_motion_processes_still_running=running,
    source_full_run_remains_failed=True,
    whole_uninterrupted_transfer_verified=False,
    new_robot_commands_sent=False,
    warning='Recorded physical state after last commanded motion, not a future live action lease. '
            'Observed table depth clearance is an estimate, not simulator ground truth. '
            'No automatic action replay or protocol response.'
)
with out.open('x') as f:
    json.dump(summary, f, indent=2, allow_nan=False)
    f.write('\n')

files = [
    'result.json', 'status.json', 'journal.jsonl', 'config.snapshot.json',
    'input-files-audit.json', 'placement-verification.json', 'placed-final-audit.json',
    'placement-plan.json', 'destination-surface.json', 'destination-navigation.json',
    'pre-release-check.json', 'placement-approach-sequence.json', 'placement-approach-00.json',
    'place-release/result.json', 'place-retract/result.json',
    'grasp/close-lift/result.json', 'backend-independent-lift-verification.json',
    'verified-held-geometry.json', 'held-after-retract-audit.json', 'held-at-destination-audit.json'
]
with tarfile.open(archive, 'x:gz') as pack:
    for name in files:
        path = run / name
        if path.exists():
            pack.add(path, arcname='trial/' + name)
    for dirname in ('destination-outside', 'placed-final'):
        captures = sorted((run / dirname).glob('*/snapshot.json'))
        selected = {captures[0].parent, captures[len(captures)//2].parent, captures[-1].parent}
        for capture in sorted(selected):
            for name in ('rgb.png', 'hand.png', 'snapshot.json'):
                pack.add(capture / name, arcname='trial/' + str((capture / name).relative_to(run)))
    pack.add(out, arcname=out.name)

print(json.dumps({k:v for k,v in summary.items()
    if k not in ('final_snapshot','destination_navigation')}, allow_nan=False), flush=True)
print(json.dumps(dict(archive=str(archive), audit=str(out)), allow_nan=False), flush=True)

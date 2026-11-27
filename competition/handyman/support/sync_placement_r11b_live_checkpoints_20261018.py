"""Back up and sync five explicitly named live-progress files; no robot control."""
import hashlib
import json
from pathlib import Path
import tarfile

base = Path('/__HANDYMAN_PACKAGE__/runtime')
archive = base / 'apple-placement-policy-r11b-live-20261018-checkpoints.tar.gz'
backup = base / 'apple-before-placement-policy-r11b-live-20261018-checkpoints.tar.gz'
names = {
    'apple-placement-policy-live-progress-20261018.json',
    'apple-placement-policy-progress-20261018.json',
    'phase5-restart-checkpoint-20261017.json',
    'apple-transfer-automation-progress-20261017.json',
    'apple-transfer-live-progress-20261017.json'
}
roots = dict(
    runs=base,
    runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),
    training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1')
)
with tarfile.open(archive, 'r:gz') as patch:
    members = patch.getmembers()
    if len(members) != 5 or {m.name for m in members} != names or any(not m.isfile() for m in members):
        raise ValueError('explicit_five_checkpoint_set_required')
    parsed = {m.name: json.loads(patch.extractfile(m).read()) for m in members}
    latest = parsed['apple-placement-policy-live-progress-20261018.json']
    if latest['new_policy_live_placement_verified'] is not True or latest['full_uninterrupted_transfer_verified'] is not False:
        raise ValueError('live_vs_full_result_must_remain_distinct')
    with tarfile.open(backup, 'x:gz') as old:
        for key, root in roots.items():
            for member in members:
                if (root / member.name).exists():
                    old.add(root / member.name, arcname=key + '/' + member.name)
    expected = {m.name: hashlib.sha256(patch.extractfile(m).read()).hexdigest() for m in members}
    for root in roots.values():
        patch.extractall(root, members=members)
        if any(hashlib.sha256((root / name).read_bytes()).hexdigest() != digest for name, digest in expected.items()):
            raise ValueError('checkpoint_copy_hash_mismatch')
    print(json.dumps(dict(status='r11b_live_checkpoint_synced', copies=len(roots), hashes=expected,
        backup=str(backup), robot_commands_sent=False)), flush=True)

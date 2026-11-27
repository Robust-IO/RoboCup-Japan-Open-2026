"""Back up/sync five progress files only; exact previous hashes protect user edits."""
import hashlib
import json
from pathlib import Path
import tarfile

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
archive=base/'apple-near-geometry-r12b-20261018-checkpoints.tar.gz'
backup=base/'apple-before-near-geometry-r12b-20261018-checkpoints.tar.gz'
previous={
    'apple-near-geometry-progress-20261018.json':None,
    'apple-placement-policy-live-progress-20261018.json':'3bc9b6b25f5edc687b8ab173ac36f303ad42d47007723501a8ed8234b87ca3c2',
    'phase5-restart-checkpoint-20261017.json':'7f68bf773e1a3e578affa1369d555b20fb2ff68bd0f2ef8e3984cd7f4a9a5cdb',
    'apple-transfer-automation-progress-20261017.json':'74c9b5c0fa5323cda20d7fbfed099caf3023f8d49a6c7c182f0dcf3961cdaca9',
    'apple-transfer-live-progress-20261017.json':'6e808b2359501f4e8d0f9744820ffc4375cc974309612a465deb7f82e89be5e3'
}
roots=dict(runs=base,runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),
    training=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training'),
    module=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/automation/apple_transfer_v1'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for root in roots.values():
    for name,expected in previous.items():
        if (sha(root/name) if (root/name).exists() else None)!=expected:raise ValueError('metadata_changed_since_last_sync:'+str(root/name))
with tarfile.open(archive,'r:gz') as patch:
    members=patch.getmembers()
    if len(members)!=5 or {m.name for m in members}!=set(previous) or any(not m.isfile() for m in members):
        raise ValueError('explicit_checkpoint_set_required')
    parsed={m.name:json.loads(patch.extractfile(m).read()) for m in members}
    latest=parsed['apple-near-geometry-progress-20261018.json']
    if latest['new_r12_live_motion_validated'] is not False or latest['robot_commands_sent_this_revision'] is not False:
        raise ValueError('code_only_update_must_not_claim_motion')
    with tarfile.open(backup,'x:gz') as old:
        for key,root in roots.items():
            for name in previous:
                if (root/name).exists():old.add(root/name,arcname=key+'/'+name)
    expected={m.name:hashlib.sha256(patch.extractfile(m).read()).hexdigest() for m in members}
    for root in roots.values():
        patch.extractall(root,members=members)
        if any(sha(root/name)!=value for name,value in expected.items()):raise ValueError('checkpoint_sync_hash_mismatch')
    print(json.dumps(dict(status='near_geometry_r12b_checkpoints_synced',copies=4,hashes=expected,
        backup=str(backup),robot_commands_sent=False)),flush=True)

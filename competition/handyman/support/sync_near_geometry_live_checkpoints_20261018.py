"""Back up/sync five progress files only; exact previous hashes protect user edits."""
import hashlib
import json
from pathlib import Path
import tarfile

base=Path('/__HANDYMAN_PACKAGE__/runtime')
archive=base/'apple-near-geometry-live-20261018-checkpoints.tar.gz'
backup=base/'apple-before-near-geometry-live-20261018-checkpoints.tar.gz'
previous={
    'apple-near-geometry-live-progress-20261018.json':None,
    'apple-near-geometry-progress-20261018.json':'280237acb6320cfe8820174d80dde5415c7cb2a307c70fe51fcbe0841d043a06',
    'phase5-restart-checkpoint-20261017.json':'880f80d209b6eda53906ff1eb5dd69afeeb213a04c5995acbb79a628647faaa3',
    'apple-transfer-automation-progress-20261017.json':'f347b5fb2fee439a3c386b02a62b52f12f4283175987d49606ca90b78404da54',
    'apple-transfer-live-progress-20261017.json':'43a5138b0e073b39fe1bd996b8f40de366c9a8e8a466cd26cfd12c0f1beddfaf'
}
roots=dict(runs=base,runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),
    training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for root in roots.values():
    for name,expected in previous.items():
        if (sha(root/name) if (root/name).exists() else None)!=expected:raise ValueError('metadata_changed_since_last_sync:'+str(root/name))
with tarfile.open(archive,'r:gz') as patch:
    members=patch.getmembers()
    if len(members)!=5 or {m.name for m in members}!=set(previous) or any(not m.isfile() for m in members):
        raise ValueError('explicit_checkpoint_set_required')
    parsed={m.name:json.loads(patch.extractfile(m).read()) for m in members}
    latest=parsed['apple-near-geometry-live-progress-20261018.json']
    if (latest['full_run']['near_geometry_reached'] is not False
            or latest['full_run']['grasp_or_place_commands_sent'] is not False
            or latest['last_recorded_robot_state_not_live_lease']['room']!='bedroom'):
        raise ValueError('source_pairing_stop_must_not_claim_grasp_or_handoff')
    with tarfile.open(backup,'x:gz') as old:
        for key,root in roots.items():
            for name in previous:
                if (root/name).exists():old.add(root/name,arcname=key+'/'+name)
    expected={m.name:hashlib.sha256(patch.extractfile(m).read()).hexdigest() for m in members}
    for root in roots.values():
        patch.extractall(root,members=members)
        if any(sha(root/name)!=value for name,value in expected.items()):raise ValueError('checkpoint_sync_hash_mismatch')
    print(json.dumps(dict(status='near_geometry_live_stop_checkpoints_synced',copies=4,hashes=expected,
        backup=str(backup),robot_commands_sent=False)),flush=True)

"""Back up/synchronize validated progress metadata only; no robot imports."""
import hashlib
import json
from pathlib import Path
import tarfile

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
archive=base/'apple-gate-pruning-r9b-20261018-checkpoints.tar.gz'
backup=base/'apple-before-gate-pruning-checkpoints-20261018.tar.gz'
roots=dict(runs=base,runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),
    training=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training'),
    module=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/automation/apple_transfer_v1'))
with tarfile.open(archive,'r:gz') as patch:
    members=patch.getmembers()
    if len(members)!=5 or any(not m.isfile() or Path(m.name).name!=m.name or not m.name.endswith('.json') for m in members):
        raise ValueError('only_five_explicit_checkpoint_json_files_allowed')
    for member in members:json.loads(patch.extractfile(member).read())
    if backup.exists():raise ValueError('checkpoint_backup_exists_no_repeat')
    with tarfile.open(backup,'x:gz') as old:
        for key,root in roots.items():
            for member in members:
                if (root/member.name).exists():old.add(root/member.name,arcname=key+'/'+member.name)
    expected={m.name:hashlib.sha256(patch.extractfile(m).read()).hexdigest() for m in members}
    for root in roots.values():
        patch.extractall(root,members=members)
        if any(hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest for name,digest in expected.items()):
            raise ValueError('checkpoint_copy_hash_mismatch')
    print(json.dumps(dict(status='five_checkpoint_json_files_validated_and_synced',copies=len(roots),
        hashes=expected,backup=str(backup),robot_commands_sent=False)),flush=True)

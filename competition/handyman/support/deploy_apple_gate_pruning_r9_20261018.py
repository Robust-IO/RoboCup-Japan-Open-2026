"""Inspect/backup/synchronize only this reviewed r9 patch; never runs robot code."""
import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import tarfile

parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true')
parser.add_argument('--revision',choices=['r9','r9b'],default='r9');args=parser.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime')
patch=base/f'apple-gate-pruning-{args.revision}-20261018-patch.tar.gz'
roots=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),
    training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
backup=base/f'apple-before-gate-pruning-{args.revision}-20261018.tar.gz'
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
with tarfile.open(patch,'r:gz') as archive:
    members=archive.getmembers()
    if not members or any(not m.isfile() or Path(m.name).name!=m.name or m.name.startswith('.') for m in members):
        raise ValueError('only_explicit_flat_regular_patch_files_allowed')
    mismatches=[];before={}
    for member in members:
        before[member.name]={key:digest(root/member.name) if (root/member.name).exists() else None for key,root in roots.items()}
        hashes={v for v in before[member.name].values() if v is not None}
        if len(hashes)>1:mismatches.append(member.name)
    if mismatches:
        print(json.dumps(dict(status='source_copies_differ_no_write',mismatches=mismatches,before=before)),flush=True)
        raise SystemExit(1)
    if not args.apply:
        print(json.dumps(dict(status='copies_consistent_ready_for_backup',patch_sha256=digest(patch),before=before)),flush=True)
        raise SystemExit(0)
    with Path('/tmp/handyman-apple-transfer-automation.lock').open('a+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if backup.exists():raise ValueError('backup_already_exists_no_repeat_deployment')
        with tarfile.open(backup,'x:gz') as old:
            for key,root in roots.items():
                for member in members:
                    if (root/member.name).exists():old.add(root/member.name,arcname=key+'/'+member.name)
        expected={m.name:hashlib.sha256(archive.extractfile(m).read()).hexdigest() for m in members}
        for root in roots.values():
            archive.extractall(root,members=members)
        after={key:{name:digest(root/name) for name in expected} for key,root in roots.items()}
        if any(value!=expected for value in after.values()):raise ValueError('deployment_hash_mismatch')
        record=dict(status='gate_pruning_r9_deployed_no_robot_commands',patch=str(patch),patch_sha256=digest(patch),
            backup=str(backup),before=before,after=after,robot_commands_sent=False,
            model_or_maps_or_navigation_parameters_changed=False,old_results_modified=False)
        with (base/f'apple-gate-pruning-{args.revision}-20261018-deployment.json').open('x') as stream:json.dump(record,stream,indent=2)
        print(json.dumps(dict(status=record['status'],files=len(expected),copies=len(roots),
            patch_sha256=record['patch_sha256'],backup=str(backup))),flush=True)

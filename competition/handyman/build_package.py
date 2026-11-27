#!/usr/bin/env python3
"""Generate an isolated competition package. Never overwrite a live package."""
import argparse,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path

REPO=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
SKIP={'runs','audit','logs','__pycache__','.tmp','.git'}

def safe_relative(value):
    p=Path(value)
    if p.is_absolute() or '..' in p.parts:raise ValueError('unsafe_manifest_path:'+value)
    return p

def build(output, runtime_base=None):
    output=Path(output).resolve()
    if output.exists():raise ValueError('output_must_not_exist:'+str(output))
    if output==REPO or REPO.is_relative_to(output) or output.is_relative_to(HERE):
        raise ValueError('output_overlaps_source')
    spec=json.loads((HERE/'package_manifest.json').read_text())
    entries=spec['files'];seen=set()
    for entry in entries:
        source=REPO/safe_relative(entry['source']);dest=safe_relative(entry['destination'])
        if dest in seen:raise ValueError('duplicate_destination:'+str(dest))
        seen.add(dest)
        if not source.is_file():raise FileNotFoundError(source)
    if runtime_base:
        base=Path(runtime_base).resolve()
        if not (base/'.package-root.json').is_file():raise ValueError('runtime_base_is_not_a_package')
        if output.is_relative_to(base) or base.is_relative_to(output):raise ValueError('output_overlaps_runtime_base')
        def ignore(directory,names):
            return [n for n in names if n in SKIP or n.endswith(('.log','.sock','.pid')) or n in ('PACKAGING_IN_PROGRESS','.package-paths.lock')]
        # copy2 creates independent files: relocation cannot modify the seed.
        shutil.copytree(base,output,symlinks=True,ignore=ignore,copy_function=shutil.copy2)
    else:output.mkdir(parents=True)
    (output/'PACKAGING_IN_PROGRESS').write_text('Run check_competition.sh before use.\n')
    if runtime_base:
        # Relocate dependencies before overwriting code with canonical sources.
        subprocess.run([sys.executable,str(output/'resolve_package_paths.py')],check=True)
    checksums={}
    for entry in entries:
        source=REPO/entry['source'];target=output/entry['destination']
        raw=source.read_bytes()
        try:raw=raw.decode('utf-8').replace(spec['path_token'],str(output)).encode('utf-8')
        except UnicodeDecodeError:pass
        target.parent.mkdir(parents=True,exist_ok=True)
        # Never write through a dependency seed's symlink.
        if target.is_symlink():target.unlink()
        target.write_bytes(raw);shutil.copymode(source,target)
        checksums[entry['destination']]=hashlib.sha256(raw).hexdigest()
    for name in ('runs','audit','.tmp'): (output/name).mkdir(exist_ok=True)
    (output/'.package-root.json').write_text(json.dumps({'root':str(output)}))
    (output/'source-build.json').write_text(json.dumps({'source_root':str(REPO),'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),'runtime_base':str(runtime_base) if runtime_base else None,'files':checksums},indent=2))
    if runtime_base:
        result=subprocess.run(['bash',str(output/'check_competition.sh')],cwd=output,capture_output=True,text=True)
        (output/'package-check.log').write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError('package_check_failed:'+str(output/'package-check.log'))
        (output/'PACKAGING_IN_PROGRESS').unlink()
    return {'output':str(output),'source_files':len(entries),'runtime_checked':bool(runtime_base),'robot_commands_sent':False}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--runtime-base',type=Path,help='Existing local package supplying model, Python/ROS environments and compiled bridge; copied, never modified')
    args=p.parse_args();print(json.dumps(build(args.output,args.runtime_base),ensure_ascii=False))
if __name__=='__main__':main()

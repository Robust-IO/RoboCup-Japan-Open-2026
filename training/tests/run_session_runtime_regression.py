"""Offline-only regression and evidence archive; requires isolated ROS domain 73."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--unit-python',default=sys.executable)
    p.add_argument('--ctest-build',type=Path,required=True)
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('isolated domain 73 / localhost required')
    root=Path(__file__).resolve().parents[2]
    a.output.mkdir(parents=True,exist_ok=False)
    results=[]
    def run(name,command,timeout=90):
        print('START',name,flush=True)
        with (a.output/(name+'.log')).open('x') as log:
            process=subprocess.run(command,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=timeout)
        row=dict(name=name,returncode=process.returncode)
        results.append(row)
        print('END',name,process.returncode,flush=True)
        return row
    run('python-units',[a.unit_python,'-m','unittest','discover','-s','training/tests','-p','test_*.py'])
    run('cpp-regression',['ctest','--test-dir',str(a.ctest_build),'--output-on-failure'])
    run('default-disabled',[sys.executable,'training/scripts/search_session_runtime.py',
        '--package-share','/nonexistent','--worker','/nonexistent','--decoder','/nonexistent',
        '--output',str(a.output/'must-not-exist')])
    assert not (a.output/'must-not-exist').exists()
    for case in ('sequence','no_target','cancel','runtime_shutdown','observer_crash','observer_crash_orphan','worker_crash','worker_crash_early','map_change','before_send','delayed_accept','accepted_lost','recovery','recovery_unknown','reject_once','journal_write_failure'):
        row=run(case,[sys.executable,'training/tests/run_search_session_runtime.py','--case',case])
        lines=(a.output/(case+'.log')).read_text().splitlines()
        paths=[Path(line[7:]) for line in lines if line.startswith('OUTPUT ')]
        if len(paths)!=1:row['evidence_error']='missing_output_path';continue
        source=paths[0]
        if source.parent!=Path('/tmp') or not source.name.startswith('search-session-runtime-'):
            raise ValueError('unexpected evidence path')
        shutil.copytree(source,a.output/case)
        summary=source/'summary.json'
        if not summary.exists() or not json.loads(summary.read_text()).get('passed'):
            row['evidence_error']='missing_passing_summary'
        cleanup=source/'cleanup.json'
        if not cleanup.exists():row['evidence_error']='missing_cleanup_proof'
        else:
            data=json.loads(cleanup.read_text())
            if data['runtime_returncode']!=0 or data['forced_runtime_kill'] or data['remaining_children']:
                row['evidence_error']='failed_process_cleanup'
        for log in source.rglob('*.log'):
            if 'Traceback (most recent call last)' in log.read_text(errors='replace'):
                row['evidence_error']='traceback_in_child_log'
    sources={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest()
        for folder in ('training/scripts','training/tests','src/handyman_rebuild_ros2/src','src/handyman_rebuild_ros2/include')
        for f in (root/folder).rglob('*') if f.suffix in ('.py','.cpp','.hpp')}
    (a.output/'source-sha256.json').write_text(json.dumps(sources,indent=2,sort_keys=True))
    result=dict(passed=all(r['returncode']==0 and 'evidence_error' not in r for r in results),
        results=results,scope='Offline ROS lifecycle only; simulated Nav2/TF/detections; no Unity or inference accuracy acceptance',
        known_limit='Unresolved durable intents stay locked, including UNKNOWN after restart. Requires preserved journal and trustworthy Nav2 terminal responses; no physical standstill proof.')
    (a.output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':sys.exit(main())

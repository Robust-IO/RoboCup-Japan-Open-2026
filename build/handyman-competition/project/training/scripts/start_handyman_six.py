"""Isolated six-class training with durable logs; no runtime deployment."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
ROOT = REPO / 'training/runs'
RUN = 'handyman_six_seg_20260912'
DATA = Path('/home/crazylearner/handyman-datasets/train03-val03-six-v22/dataset-seg.yaml')
WEIGHTS = ROOT / 'handyman_v31_seg_20260910/weights/best.pt'
STATUS = ROOT / (RUN + '.status.json')
LOG = ROOT / (RUN + '.console.log')

def status(**fields):
    temporary = STATUS.with_suffix('.tmp')
    temporary.write_text(json.dumps(fields, indent=2))
    temporary.replace(STATUS)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', action='store_true')
    args = parser.parse_args()
    if args.worker:
        command = ['/home/crazylearner/.pixi/bin/pixi', 'run', '--frozen', '--manifest-path', str(REPO/'pixi.toml'),
                   'yolo', 'segment', 'train', 'cfg='+str(REPO/'training/handyman_pilot_seg.yaml'),
                   'model='+str(WEIGHTS), 'data='+str(DATA), 'project='+str(ROOT), 'name='+RUN,
                   'epochs=30', 'seed=20260924', 'resume=False', 'exist_ok=False']
        started = time.time()
        status(state='running', supervisor_pid=os.getpid(), started=started, command=command)
        try:
            code = subprocess.call(command, cwd=REPO)
        except Exception as exc:
            status(state='failed', started=started, finished=time.time(), error=str(exc))
            raise
        status(state='completed' if code == 0 else 'failed', started=started, finished=time.time(),
               exit_code=code, command=command, run_dir=str(ROOT/RUN))
        return code
    for p in [DATA, WEIGHTS, REPO/'training/handyman_pilot_seg.yaml']:
        if not p.is_file():
            raise SystemExit('Missing input: '+str(p))
    report = json.loads((DATA.parent/'qa_report.json').read_text())
    expected = ['apple','canned_juice','rabbit_doll','pink_cup','white_cup','filled_ketchup']
    if report['classes'] != expected or any(any(n == 0 for n in report['classCounts'][s]) for s in ['train','val']):
        raise SystemExit('Six-class QA registry/coverage failed')
    ROOT.mkdir(parents=True, exist_ok=True)
    if any(p.exists() for p in [ROOT/RUN, STATUS, LOG]):
        raise SystemExit('Existing run/log/status: refusing overwrite')
    with LOG.open('x') as log:
        child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker'], cwd=REPO,
                                 stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    print('Supervisor:', child.pid, '\nLog:', LOG)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())

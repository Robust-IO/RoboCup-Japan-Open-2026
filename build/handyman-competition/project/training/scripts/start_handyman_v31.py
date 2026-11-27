"""Durable V3.1 fine-tuning run; never deploys runtime weights."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
ROOT = REPO / 'training/runs'
RUN = 'handyman_v31_seg_20260910'
STATUS = ROOT / (RUN + '.status.json')
LOG = ROOT / (RUN + '.console.log')
DATA = Path('/home/crazylearner/handyman-datasets/train02-val02-v31/dataset-seg.yaml')
WEIGHTS = ROOT / 'handyman_pilot_seg_20260910/weights/best.pt'


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
                   'epochs=30', 'seed=20260917', 'resume=False', 'exist_ok=False']
        started = time.time()
        status(state='running', supervisor_pid=os.getpid(), started=started, command=command)
        try:
            code = subprocess.call(command, cwd=REPO)
        except Exception as exc:
            status(state='failed', error=str(exc), started=started, finished=time.time())
            raise
        status(state='completed' if code == 0 else 'failed', exit_code=code, started=started,
               finished=time.time(), run_dir=str(ROOT/RUN), command=command)
        return code
    for path in [DATA, WEIGHTS, REPO/'training/handyman_pilot_seg.yaml']:
        if not path.is_file():
            raise SystemExit('Missing required input: '+str(path))
    ROOT.mkdir(parents=True, exist_ok=True)
    if any(p.exists() for p in [ROOT/RUN, STATUS, LOG]):
        raise SystemExit('Existing run/log/status detected; refusing overwrite')
    with LOG.open('x') as log:
        process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker'], cwd=REPO,
                                   stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    print('Supervisor PID:', process.pid)
    print('Log:', LOG)
    print('Status:', STATUS)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

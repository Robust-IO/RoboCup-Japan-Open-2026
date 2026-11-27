"""Launch the first pilot with durable logs; no runtime weight deployment."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
RUN = 'handyman_pilot_seg_20260910'
ROOT = REPO / 'training/runs'
STATUS = ROOT / (RUN + '.status.json')
LOG = ROOT / (RUN + '.console.log')
DATA = Path('/home/crazylearner/handyman-datasets/train01-val01-v21/dataset-seg.yaml')


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
                   'model=yolo26n-seg.pt', 'data='+str(DATA), 'project='+str(ROOT), 'name='+RUN]
        status(state='running', supervisor_pid=os.getpid(), started=time.time(), command=command)
        try:
            code = subprocess.call(command, cwd=REPO)
        except Exception as exc:
            status(state='failed', error=str(exc), finished=time.time())
            raise
        status(state='completed' if code == 0 else 'failed', exit_code=code, finished=time.time(), run_dir=str(ROOT/RUN))
        return code
    if not DATA.is_file():
        raise SystemExit('Validated dataset YAML missing')
    ROOT.mkdir(parents=True, exist_ok=True)
    if any(p.exists() for p in [ROOT/RUN, STATUS, LOG]):
        raise SystemExit('Existing run/log/status detected. Inspect it; do not overwrite.')
    with LOG.open('x') as log:
        process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker'], cwd=REPO,
                                   stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    print('Supervisor PID:', process.pid)
    print('Log:', LOG)
    print('Status:', STATUS)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

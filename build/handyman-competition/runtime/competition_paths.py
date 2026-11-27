"""Run-output roots are independent from packaged executable locations."""
from pathlib import Path
RUN_ROOT=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runs')
LEGACY_RUN_ROOT=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
def is_run_path(value):
    path=Path(value).resolve()
    return any(path.is_relative_to(root.resolve()) for root in (RUN_ROOT,LEGACY_RUN_ROOT))

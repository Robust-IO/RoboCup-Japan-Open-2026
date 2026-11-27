"""Run-output roots are independent from packaged executable locations."""
from pathlib import Path
RUN_ROOT=Path('/__HANDYMAN_PACKAGE__/runs')
LEGACY_RUN_ROOT=Path('/__HANDYMAN_PACKAGE__/runtime')
def is_run_path(value):
    path=Path(value).resolve()
    return any(path.is_relative_to(root.resolve()) for root in (RUN_ROOT,LEGACY_RUN_ROOT))

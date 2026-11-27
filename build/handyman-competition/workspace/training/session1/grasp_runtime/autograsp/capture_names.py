"""Keep every retry capture; never overwrite the first closure's evidence."""
from pathlib import Path

def unused_capture_name(root,name):
    directory=Path(root)/'captures'
    if not (directory/name).exists():return name
    index=2
    while (directory/f'{name}-attempt-{index:03d}').exists():index+=1
    return f'{name}-attempt-{index:03d}'

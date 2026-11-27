"""Atomic completed-frame index; avoids directory-wide live scans on Windows D:."""
import json
import os
from pathlib import Path
import re


def publish(directory, capture_name, epoch, observation):
    if not re.fullmatch(r'\d{6}', capture_name) or not epoch:
        raise ValueError('invalid_latest_capture_index')
    directory=Path(directory)
    path=directory/'latest-capture.json'
    pending=directory/'latest-capture.pending'
    with pending.open('w') as stream:
        json.dump(dict(capture=capture_name,tracking_epoch=epoch,observation_complete=observation),stream)
    pending.replace(path)


def latest(directory, filename='observation.json'):
    directory=Path(directory)
    path=directory/'latest-capture.json'
    if path.exists():
        item=json.loads(path.read_text())
        name=item.get('capture','')
        if not re.fullmatch(r'\d{6}',name) or not item.get('tracking_epoch'):
            raise ValueError('invalid_latest_capture_index')
        if filename=='observation.json' and item.get('observation_complete') is not True:
            raise ValueError('latest_observation_not_complete')
        result=directory/name/filename
        if not result.is_file():raise FileNotFoundError(result)
        return result
    # Legacy recordings have no index. One directory listing, newest first;
    # do not stat every historical frame as Path.glob('*/file') does on /mnt/d.
    names=sorted((entry.name for entry in os.scandir(directory) if re.fullmatch(r'\d{6}',entry.name)),reverse=True)
    for name in names:
        result=directory/name/filename
        if result.is_file():return result
    raise FileNotFoundError('no_completed_capture:'+str(directory))

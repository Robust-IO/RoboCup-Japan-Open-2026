"""Per-run can evidence and shared controller ownership. No ROS imports."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path

import numpy as np

from apple_transfer_automation import RobotLock,write_json
from canned_grasp_geometry import lift_follow

LOCK_PATH='/tmp/handyman-apple-transfer-automation.lock'
read=lambda path:json.loads(Path(path).read_text())
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()


@contextmanager
def stage_lock(fd=None,run_root=None):
    """Borrow the parent's SAME open-file description; never unlock its lease."""
    if fd is None:
        if run_root is not None:raise ValueError('owner_fd_and_run_root_required_together')
        with RobotLock() as lock:yield lock
        return
    if fd<3 or run_root is None or not (Path(run_root)/'run-meta.json').is_file():
        raise ValueError('invalid_parent_automation_owner')
    own=os.fstat(fd);expected=os.stat(LOCK_PATH)
    if (own.st_dev,own.st_ino)!=(expected.st_dev,expected.st_ino):raise ValueError('wrong_parent_lock_descriptor')
    # An independently opened descriptor must be blocked by the parent lease.
    with open(LOCK_PATH,'a+') as probe:
        try:fcntl.flock(probe,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:pass
        else:raise ValueError('parent_automation_lock_not_held')
    duplicate=os.dup(fd)
    try:
        fcntl.flock(duplicate,fcntl.LOCK_EX|fcntl.LOCK_NB)
        yield None
    finally:os.close(duplicate)


def build_context(run_root):
    root=Path(run_root).resolve();meta=read(root/'run-meta.json');grasp=root/'grasp';result=read(grasp/'result.json')
    if (meta['object']!='canned_juice' or result.get('status')!='can_grasp_and_lift_verified'
            or result.get('grasp_success_verified') is not True or result.get('task_id')!='grasp'
            or result.get('close',{}).get('status')!='passed' or result.get('close',{}).get('grasp_closure_commanded') is not True
            or result.get('lift',{}).get('status')!='passed'):
        raise ValueError('current_run_verified_can_grasp_required')
    rows=result.get('lift_verification',[])
    if len(rows)!=3:raise ValueError('three_current_run_lift_frames_required')
    opening=result['held_opening_rad'];offsets=[];radii=[];stamps=[]
    before=Path(result['close']['before_capture']).resolve()
    if not before.is_relative_to(grasp/'captures'):raise ValueError('preclose_capture_outside_run')
    m0=read(before/'can-measurement.json');s0=read(before/'snapshot.json')
    if m0.get('class_name')!='canned_juice' or m0['stamp_ns']<meta['started_ns']:raise ValueError('current_can_preclose_required')
    for row in rows:
        path=Path(row['capture']).resolve()
        if not path.is_relative_to(grasp/'captures'):raise ValueError('held_capture_outside_run')
        m=read(path/'can-measurement.json');s=read(path/'snapshot.json')
        if m.get('class_name')!='canned_juice' or m['stamp_ns']<meta['started_ns']:
            raise ValueError('historical_or_other_object_lift_capture')
        proof=lift_follow(m0,s0,m,s,opening)
        if proof['stamp_ns']!=row['stamp_ns']:raise ValueError('lift_report_capture_mismatch')
        offsets.append(np.asarray(m['center_m'])-s['palm_base_at_depth']);radii.append(m['footprint_radius_m']);stamps.append(m['stamp_ns'])
    if any(b-a<150_000_000 for a,b in zip(stamps,stamps[1:])):raise ValueError('distinct_current_lift_frames_required')
    offset=np.mean(offsets,axis=0);height=result['reference']['height_m'];source=Path(rows[-1]['capture']).resolve()
    measurements=[read(Path(row['capture'])/'can-measurement.json') for row in rows]
    axis=np.mean([m.get('axis_base_m',[0.,0.,1.]) for m in measurements],axis=0);axis/=np.linalg.norm(axis)
    extent=max(m.get('bottom_vertical_extent_m',height/2) for m in measurements)
    support_radius=max(m.get('horizontal_support_radius_m',m['footprint_radius_m']) for m in measurements)
    if np.max(np.linalg.norm(np.asarray(offsets)-offset,axis=1))>.02:raise ValueError('unstable_current_held_offset')
    value=dict(version=1,run_id=meta['run_id'],run_root=str(root),boot_id=meta['boot_id'],class_name='canned_juice',
        can_grasp_and_lift_verified=True,held_opening_rad=opening,held_center_minus_palm_base_m=offset.tolist(),
        can_height_m=height,can_footprint_radius_m=float(np.median(radii)),
        can_axis_base_m=axis.tolist(),can_vertical_extent_m=extent,can_support_radius_m=support_radius,
        estimated_upright_bottom_below_palm_m=float(height/2-offset[2]),estimated_bottom_below_palm_m=float(extent-offset[2]),
        contact_start_lift_m=result['grasp_plan']['contact_start_lift_m'],
        source_capture=str(source),source_grasp_result=str(grasp/'result.json'),source_grasp_sha256=sha(grasp/'result.json'),
        source_capture_sha256={n:sha(source/n) for n in ('rgb.png','depth_m.npy','snapshot.json','can-measurement.json')},
        source_stamps_ns=stamps,historical_checkpoint_used=False,coordinates_are_not_new_motion_authorization=True)
    path=root/'held-context.json'
    if path.exists():raise ValueError('run_context_already_exists')
    write_json(path,value);return path


def load_context(path,output,*,require_carried=False):
    path=Path(path).resolve();value=read(path);root=Path(value['run_root']).resolve();meta=read(root/'run-meta.json')
    if (value.get('version')!=1 or value.get('class_name')!='canned_juice' or value.get('can_grasp_and_lift_verified') is not True
            or path!=root/'held-context.json' or Path(output).resolve().parent!=root or meta.get('mode')!='live'
            or value['run_id']!=meta['run_id'] or value['boot_id']!=meta['boot_id']
            or value['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()):
        raise ValueError('current_run_context_mismatch')
    result=Path(value['source_grasp_result']);source=Path(value['source_capture'])
    if result.resolve()!=root/'grasp/result.json' or not source.resolve().is_relative_to(root/'grasp/captures'):
        raise ValueError('context_source_outside_current_run')
    if (sha(result)!=value['source_grasp_sha256'] or any(sha(source/n)!=digest for n,digest in value['source_capture_sha256'].items())
            or min(value['source_stamps_ns'])<meta['started_ns']):raise ValueError('current_run_grasp_evidence_changed_or_historical')
    if require_carried:
        carried=read(root/'carry/result.json')
        if (carried.get('status')!='can_retraction_and_destination_navigation_verified'
                or not carried.get('retraction_verified') or not carried.get('destination_navigation_verified')
                or carried.get('context_run_id')!=value['run_id'] or carried.get('hand_released') is not False):
            raise ValueError('current_run_held_destination_navigation_required')
    return value

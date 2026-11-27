"""Persistent recovery waits. Cancelling the process still cancels the wait."""
import json
import math
import time
from pathlib import Path

CONTACT_STABILITY = dict(stop_base_translation_m=.003, stop_base_yaw_rad=.005,
    stop_arm_lift_m=.003, stop_arm_angle_rad=.02, stop_head_angle_rad=.02)

def transient_observation_error(error):
    text=str(error)
    return (type(error).__name__ in ('LookupException','ConnectivityException','ExtrapolationException')
        or any(token in text for token in ('stale_joints','stale_base_tf','feedback_wait_expired',
            'missing_feedback','missing_streams','waiting_for_coherent','requested_observation_unavailable',
            'observation_slice_pending','stopped_capture_not_available','new_image_after_stop_required',
            'robot_changed_since_stopped_capture','fresh_current_action_feedback_required')))

class RecoveryStatus:
    def __init__(self,root=None,record=lambda _:None,clock=time.monotonic):
        self.root=Path(root) if root else None;self.record=record;self.clock=clock
        self.last_reason=None;self.last_notice=-math.inf;self.started=None
    def waiting(self,reason):
        now=self.clock()
        if self.started is None:self.started=now
        if reason==self.last_reason and now-self.last_notice<5:return
        self.last_reason=reason;self.last_notice=now
        value=dict(status='waiting_for_recovery',reason=reason,elapsed_wait_s=now-self.started,
            automatic_task_abort=False,robot_commands_sent=False)
        if self.root:
            target=self.root/'recovery-state.json';temp=target.with_suffix('.new')
            temp.write_text(json.dumps(value,ensure_ascii=False,indent=2));temp.replace(target)
        self.record(dict(event='gate_recovery_wait',**value))
    def recovered(self):
        if self.started is None:return
        value=dict(status='recovered',reason=self.last_reason,elapsed_wait_s=self.clock()-self.started)
        if self.root:(self.root/'recovery-state.json').write_text(json.dumps(value,indent=2))
        self.record(dict(event='gate_recovery_complete',**value));self.started=None;self.last_reason=None

def room_for_pose(pose,rooms,inside,boundary_distance,*,previous=None,preferred=None):
    """5cm confident interior; resolve doorway uncertainty without a 15cm veto."""
    owners=[name for name,room in rooms.items() if inside(*pose[:2],room['region'])]
    interior=[name for name in owners if boundary_distance(*pose[:2],rooms[name]['region'])>.05]
    if len(interior)==1:return interior[0]
    if len(owners)==1:return owners[0]
    for hint in (preferred,previous):
        if hint in owners:return hint
    if not owners:
        close=[name for name,room in rooms.items() if boundary_distance(*pose[:2],room['region'])<=.05]
        for hint in (previous,preferred):
            if hint in close:return hint
        if len(close)==1:return close[0]
    raise ValueError('room_assignment_needs_current_route_context')

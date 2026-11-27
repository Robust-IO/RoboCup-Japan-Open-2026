"""Measured door arrival and escalation of repeated no-progress navigation."""
from navigation_turn_policy import final_pose_ready

def door_pose_ready(row,pose,acceptance):
    return row['kind'] in ('door_entry','door_exit') and final_pose_ready(pose,row['pose'],*acceptance)

def needs_escape(pending):
    return pending.get('retry',0)>=3 and pending.get('progress_m',float('inf'))<.02

"""Separate measured post-endpoint camera frames from a moving verification lift."""


def bounds(trial):
    start=trial.get('lift_completed_at_ns');end=trial.get('verification_window_end_ns')
    if (type(start) is not int or type(end) is not int or start<=0 or end<=start):
        raise ValueError('actual_lift_completion_and_frozen_window_required')
    return start,end


def contains(snapshot,start,end):
    # A latest joint endpoint can accompany an older head image. Never infer
    # camera settling from that joint value alone, or renew either timestamp.
    streams=snapshot['streams'];keys=['rgb','depth']+(['hand'] if 'hand' in streams else [])
    stamps=[streams[key]['stamp_ns'] for key in keys]
    if any(type(value) is not int or value<=0 for value in stamps):raise ValueError('invalid_original_camera_stamp')
    return min(stamps)>=start and max(stamps)<=end

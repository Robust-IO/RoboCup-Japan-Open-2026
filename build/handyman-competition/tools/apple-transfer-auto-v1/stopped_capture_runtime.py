"""Class-independent stopped acquisition with each adapter's motion preserved."""
from outcome_transfer_runtime import OutcomeRuntime

def with_stopped_capture(adapter,*,profile='rgbd'):
    from observation_profiles import channels
    channels(profile)
    class StoppedCaptureRuntime(OutcomeRuntime,adapter):
        default_observation_profile=profile
        if hasattr(adapter,'move_joints'):move_joints=adapter.move_joints
        if hasattr(adapter,'close_gripper'):close_gripper=adapter.close_gripper
    StoppedCaptureRuntime.__name__='Stopped'+adapter.__name__
    return StoppedCaptureRuntime

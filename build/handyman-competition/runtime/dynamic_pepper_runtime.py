"""Reuse proven stopped RGBD acquisition; preserve pepper motion/contact logic."""
from outcome_transfer_runtime import OutcomeRuntime
from pepper_runtime import PepperRuntime

class DynamicPepperRuntime(OutcomeRuntime,PepperRuntime):
    default_observation_profile='rgbd'
    # Keep the actual pepper arm/closure methods and their event names. The
    # mixin supplies stopped capture, current-TF snapshots, and sensor recovery.
    move_joints=PepperRuntime.move_joints
    close_gripper=PepperRuntime.close_gripper

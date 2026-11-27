"""Diagnostic pauses around the unchanged current can closure/lift actions.

No geometry, thresholds, targets, motion speed or acceptance is overridden.
The original closure reacquires a fresh measurement AFTER the viewing pause.
"""
import sys
import time
from pathlib import Path

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
import canned_grasp_runtime

class VisibleCanRuntime(canned_grasp_runtime.CanRuntime):
    def viewing_pause(self,phase):
        self.record(dict(event='human_viewing_pause',phase=phase,duration_s=6.,robot_commands_sent=False))
        finish=time.monotonic()+6.
        while time.monotonic()<finish:self.pump(.02)
        self.record(dict(event='human_viewing_pause_finished',phase=phase))

    def close_gripper(self,observe,expected_arm):
        self.viewing_pause('before_closure_hand_open')
        return super().close_gripper(observe,expected_arm)

    def record_closed_pose(self,reference,task_id):
        rows=super().record_closed_pose(reference,task_id)
        self.viewing_pause('after_closure_before_lift')
        return rows

canned_grasp_runtime.CanRuntime=VisibleCanRuntime
from run_canned_grasp_trial import main
main()

import unittest
from types import SimpleNamespace
import numpy as np
from grasp_runtime.autograsp.ros_backend import RosBackend
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure

class RotationApproachTests(unittest.TestCase):
    def backend(self,fail_height=False):
        calls=[];j=[.5,-1.57,0.,-1.57,0.,.65]
        def arm(goal,label):
            calls.append(label)
            if fail_height and label=='rotation_retry_complete_contact_height':
                raise AttemptFailure(Failure.MISALIGNED)
            j[:]=goal
        b=SimpleNamespace(check_task_cancelled=lambda:None,holding_state=lambda:'empty',
            rt=SimpleNamespace(state=lambda:{'joints':j.copy()},contact_error=lambda *a:np.array([.02,.01,-.08])),
            lift_done=False,rotation_cycle_joints=j.copy(),rotation_cycle_candidate={'arm_goal':j.copy()},
            _sample_for_path=lambda:None,_arm=arm,
            _base=lambda delta,label:calls.append(label),contact_attempt_started=False,
            _observe_tracking=lambda:({'snapshot':{}},{'center_m':[0,0,0]}),
            _finish_grasp=lambda *a:calls.append('close'))
        b._complete_approach_after_rotation=lambda:RosBackend._complete_approach_after_rotation(b)
        return b,calls,j
    def test_rotation_then_alignment_descent_then_close(self):
        b,c,j=self.backend();RosBackend.rotate_only_retry(b,1)
        self.assertEqual(c,['rotation_only_retry_1','rotation_retry_restore_approach_posture',
            'rotation_retry_complete_alignment','rotation_retry_complete_contact_height','close'])
        self.assertAlmostEqual(j[0],.42)
        self.assertTrue(b.contact_attempt_started)
    def test_failed_descent_does_not_close_or_mark_contact(self):
        b,c,j=self.backend(True)
        with self.assertRaises(AttemptFailure):RosBackend.rotate_only_retry(b,1)
        self.assertNotIn('close',c);self.assertFalse(b.contact_attempt_started)

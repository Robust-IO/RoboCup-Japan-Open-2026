import unittest
from release_record import record_release
class ReleaseTests(unittest.TestCase):
    def test_sensor_clock_not_wall_clock(self):
        r=record_release({'streams':{'depth':{'stamp_ns':123}},'joints':{'hand_motor_joint':.65}},session_id='s',run_id='r',operation_id='r/7',hand_is_open=lambda x:x>=.6)
        self.assertEqual(r['release_stamp_ns'],123);self.assertFalse(r['placement_verified'])
    def test_closed_hand_rejected(self):
        with self.assertRaises(ValueError):
            record_release({'streams':{'depth':{'stamp_ns':123}},'joints':{'hand_motor_joint':.3}},session_id='s',run_id='r',operation_id='r/7',hand_is_open=lambda x:x>=.6)

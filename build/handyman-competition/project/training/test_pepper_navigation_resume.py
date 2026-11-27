import copy,math,unittest
from pathlib import Path
from unittest.mock import Mock
from pepper_transfer_automation import validate_navigation_hold
from apple_transfer_auto_backend import LiveBackend

class ResumeTests(unittest.TestCase):
    def setUp(self):
        self.previous=dict(status='stopped_requires_actual_state_review',class_name='ground_pepper',stage='navigate_destination',
            grasp_success_verified=True,close_commands_sent=1,release_commands_sent=0,confirmed_stages=['grasp_lift'])
        self.metadata=dict(boot_id='current',run_id='old',run_root='old-root')
        self.held=dict(self.metadata,class_name='ground_pepper',grasp_lift_verified=True,holding_opening_rad=.4)
        self.state=dict(joints=[0.,0.,0.,-math.pi/2,0.,.4])
        self.route=[[6.99,-6.24,3.14],[5.59,-6.24,1.63],[2.05,2.55,1.57]]
        self.nav=dict(status='failed',nav_reset=True,speed_parameters_restored=True,error='stale_navigation_feedback:scan',
            target=self.route[-1],reached=[dict(index=0,target=self.route[0],status=4)])
    def validate(self):return validate_navigation_hold(self.previous,self.metadata,self.held,self.state,'current',self.nav,self.route)
    def test_remaining_prefix_never_replays_completed_waypoint(self):self.assertEqual(self.validate(),self.route[1:-1])
    def test_wrong_boot_grip_or_released_history_rejected(self):
        self.state['joints'][5]=.65
        with self.assertRaises(ValueError):self.validate()
        self.state['joints'][5]=.4;self.previous['release_commands_sent']=1
        with self.assertRaises(ValueError):self.validate()
        self.previous['release_commands_sent']=0;self.held['boot_id']='another'
        with self.assertRaises(ValueError):self.validate()
    def test_changed_route_or_wrong_failure_not_replayed(self):
        self.nav['reached'][0]['target']=[99.,99.,0.]
        with self.assertRaisesRegex(ValueError,'prefix'):self.validate()
        self.nav['reached']=[];self.nav['error']='carry_pose_changed:hand_motor_joint'
        with self.assertRaisesRegex(ValueError,'unreleased'):self.validate()
    def test_navigation_recovery_is_explicit_and_uses_only_remaining_waypoints(self):
        backend=LiveBackend.__new__(LiveBackend);backend.output=Path('/tmp/reviewed-output');backend.code=Path('/tmp/code')
        backend.config=dict(motion_profile='balanced_v1',navigation_joint_feedback_max_age_s=.85,navigation_feedback_recovery=True)
        backend.route=Mock(side_effect=AssertionError('completed route must not be replayed'))
        backend.command=Mock(return_value=dict(nav_reset=True,goal_precision_changed=False,speed_parameters_restored=True,grip_commands_sent=False))
        backend.navigate('bedroom','living_room',self.route[-1],.4,'remaining',remaining_waypoints=self.route[1:-1])
        args=backend.command.call_args.args[0]
        self.assertIn('--recover-feedback-interruption',args);self.assertEqual(args.count('--waypoint'),1)
        self.assertEqual(args[args.index('--waypoint')+1:],self.route[1])
        backend.config={};backend.navigate('bedroom','living_room',self.route[-1],.4,'remaining',remaining_waypoints=[])
        self.assertNotIn('--recover-feedback-interruption',backend.command.call_args.args[0])

if __name__=='__main__':unittest.main()

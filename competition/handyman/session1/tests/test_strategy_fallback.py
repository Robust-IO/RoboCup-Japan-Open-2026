import unittest
from types import SimpleNamespace
from unittest.mock import patch
from grasp_runtime.autograsp.coordinator import Coordinator,AttemptFailure,LiftEvidence
from grasp_runtime.autograsp.recovery import Failure
from grasp_runtime.autograsp.planner import Policy

class StrategyFallbackTests(unittest.TestCase):
    def run_case(self,failure):
        calls=[];policies=[];actions=[];events=[]
        b=SimpleNamespace(use_family_templates=True,policy=Policy(),holding_state=lambda:'empty',
            observe_stopped=lambda target:SimpleNamespace(stamp_ns=len(calls)+1),
            verify_lift=lambda:LiftEvidence(True,False,True,3,'lift_and_reclose_joint_feedback',.08,True),
            recover=actions.append,stop=lambda:None)
        def execute(candidate,scene):
            calls.append(candidate['strategy'])
            if len(calls)==1:raise AttemptFailure(failure)
        b.execute_candidate=execute
        def planner(scene,hand,policy,excluded):
            policies.append(policy)
            return {'selected':{'id':str(len(calls)), 'strategy':policy.preferred_strategy,'score':1,'grasp_point_base_m':[0,0,0]}}
        # A failed narrow side grasp must not override the new side observation.
        with patch('grasp_runtime.autograsp.coordinator.plan',side_effect=planner),patch(
                'grasp_runtime.autograsp.templates.select_template',return_value=(Policy(),{})):
            result=Coordinator(b,None,record=events.append).run('canned_juice')
        return calls,policies,actions,result
    def test_side_failure_alone_keeps_side_grasp(self):
        for f in (Failure.EMPTY_CLOSE,Failure.SLIPPED,Failure.MISALIGNED,Failure.APPROACH_COLLISION):
            calls,p,a,r=self.run_case(f)
            self.assertEqual(calls,['side','side'])
            self.assertIn('side',p[-1].allowed_strategies)
            self.assertNotEqual(a[0].get('strategy_next'),'overhead')
            self.assertEqual(r['status'],'grasp_verified')
    def test_sensor_failure_does_not_force_strategy_change(self):
        calls,_,_,_=self.run_case(Failure.SENSOR_UNAVAILABLE)
        self.assertEqual(calls,['side','side'])

import unittest
from types import SimpleNamespace
from table_withdrawal import retreat_distance,TableWithdrawal,PlacementStandoff
class WithdrawalTests(unittest.TestCase):
    def test_measured_distance_no_fixed_steps(self):
        self.assertAlmostEqual(retreat_distance([[.8,0,.5]],[1,0],[.5,.5],[1,0]),.33)
        self.assertEqual(retreat_distance([[.4,0,.5]],[1,0],[.5,.5],[1,0]),0)
    def test_rotated_heading(self):
        self.assertAlmostEqual(retreat_distance([[0,.8,.5]],[0,1],[.5,.5],[0,1]),.33)
    def test_remeasure_after_full_retreat_preserves_opening(self):
        rows=iter([dict(points_map=[[.8,0,.5]],table_center=[1,0],table_half=[.5,.5],heading=[1,0],evidence_ref='a'),
                   dict(points_map=[[.4,0,.5]],table_center=[1,0],table_half=[.5,.5],heading=[1,0],evidence_ref='b')])
        calls=[];rt=SimpleNamespace(state=lambda:dict(joints=[0,0,0,0,0,.32],poses={'map':[0,0,0]}))
        backend=SimpleNamespace(adjust=lambda *a:calls.append(a))
        result=TableWithdrawal(rt,backend,lambda:next(rows))()
        self.assertTrue(result['clear_to_fold']);self.assertAlmostEqual(calls[0][2][0],-.33)
        self.assertEqual(calls[0][-1],.32);self.assertEqual(len(calls),1)
    def test_standoff_replans_after_retreat(self):
        checks=iter([dict(ready=False,minimum_clearance_m=.03,clearance_m=-.1),dict(ready=True)])
        calls=[];rt=SimpleNamespace(state=lambda:dict(joints=[0,0,0,0,0,.32],poses={'map':[0,0,0]}))
        plan=dict(palm_height_m=.5,held_center_minus_palm_m=[0,0,0])
        obs=SimpleNamespace(runtime=rt,observe=lambda label:dict(item={'snapshot':{}},plane={}),measure_and_plan=lambda:dict(plan,replanned=True))
        result=PlacementStandoff(obs,SimpleNamespace(adjust=lambda *a:calls.append(a)),clearance=lambda *a:next(checks),predict=lambda *a:[0,0,0],map_point=lambda s,p:p)(plan)
        self.assertTrue(result['replanned']);self.assertAlmostEqual(calls[0][2][0],-.15)

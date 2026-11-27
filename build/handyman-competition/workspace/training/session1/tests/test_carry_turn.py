import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from candidate_selection import CandidateSelector
from carry_clearance import CarryClearance
from carry_navigation import CarryNavigation

class Turns(unittest.TestCase):
    def geometry(self,obstacles=()):
        free=np.ones((160,160),bool)
        for y,x in obstacles:free[y:y+4,x:x+4]=False
        return CarryClearance(CandidateSelector(free,.025,[0,0,0],.2,{}),
                              np.array([[.65,-.04,.3],[.65,.04,.3]]))
    def test_left_long_way_when_short_right_hits_furniture(self):
        p=self.geometry([(65,92)]).choose_turn([2,2,0],-math.pi/2)
        self.assertEqual(p['direction'],'left');self.assertGreater(p['delta_rad'],math.pi)
        self.assertFalse(p['alternatives'][1]['clear'])
    def test_right_long_way_when_short_left_hits_furniture(self):
        p=self.geometry([(92,92)]).choose_turn([2,2,0],math.pi/2)
        self.assertEqual(p['direction'],'right');self.assertLess(p['delta_rad'],-math.pi)
    def test_both_blocked(self):
        with self.assertRaisesRegex(ValueError,'no_clear_carry_turn_arc'):
            self.geometry([(65,92),(92,92)]).choose_turn([2,2,0],math.pi)
    def test_open_area_uses_short_path(self):
        self.assertAlmostEqual(self.geometry().choose_turn([2,2,0],-.4)['delta_rad'],-.4)
    def test_zero_and_invalid(self):
        g=self.geometry();self.assertEqual(g.choose_turn([2,2,0],2*math.pi)['delta_rad'],0)
        with self.assertRaises(ValueError):g.choose_turn([2,2,0],float('nan'))
    def test_execution_keeps_left_sign_and_measures_completion(self):
        g=self.geometry([(65,92)]);pose=[2.,2.,0.];commands=[]
        rt=SimpleNamespace(sample=lambda **kw:None,state=lambda:dict(poses={'map':list(pose)},joints=[0]*5+[.3]))
        def adjust(name,start,delta,arm,opening):
            commands.append(delta);pose[2]=math.atan2(math.sin(pose[2]+delta[2]),math.cos(pose[2]+delta[2]))
        with tempfile.TemporaryDirectory() as d:
            stages=SimpleNamespace(runtime=rt,output=Path(d),check=lambda cancel:None,motion=SimpleNamespace(adjust=adjust))
            CarryNavigation(stages).turn(g,-math.pi/2,None)
        self.assertTrue(all(0<x[2]<=.3 for x in commands));self.assertAlmostEqual(pose[2],-math.pi/2)
    def test_stuck_rotation_stops_instead_of_looping(self):
        rt=SimpleNamespace(sample=lambda **kw:None,state=lambda:dict(poses={'map':[2,2,0]},joints=[0]*5+[.3]))
        with tempfile.TemporaryDirectory() as d:
            stages=SimpleNamespace(runtime=rt,output=Path(d),check=lambda cancel:None,motion=SimpleNamespace(adjust=lambda *a:None))
            with self.assertRaisesRegex(ValueError,'feedback_mismatch'):CarryNavigation(stages).turn(self.geometry(),.4,None)

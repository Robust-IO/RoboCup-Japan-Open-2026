import ast
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import search_arrival_ros


class RosBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.key='0123456789abcdef0123456789abcdef'
        self.gate=search_arrival_ros.GoalStatusGate(self.key)

    def test_old_success_not_accepted(self):
        self.assertIsNone(self.gate.consume(self.key,4))

    def test_matching_active_then_success(self):
        self.gate.consume(self.key,2)
        self.assertIs(self.gate.consume(self.key,4),True)
        self.assertIsNone(self.gate.consume(self.key,4))

    def test_wrong_goal(self):
        self.gate.consume('f'*32,2)
        self.assertIsNone(self.gate.consume(self.key,4))

    def test_failure(self):
        self.gate.consume(self.key,1)
        self.assertIs(self.gate.consume(self.key,6),False)

    def test_no_command_apis(self):
        tree=ast.parse(Path(search_arrival_ros.__file__).read_text())
        attrs={n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute)}
        self.assertFalse(attrs & {'create_publisher','create_client','send_goal_async','cancel_goal_async'})

if __name__=='__main__': unittest.main()

import unittest
from session1 import Session1,INSTRUCTION
from executor import Executor
class Port:
    def __init__(self,result,history):self.result=result;self.history=history;self.pending=False
    def start(self,e):self.history.append(e.get('stage',e.get('message')));return e
    def poll(self,h):return None if self.pending else self.result
    def cancel(self,h,**kw):self.history.append(('cancel',kw['preserve_grip']))
class ExecutorTests(unittest.TestCase):
    def flow(self):
        s=Session1('test',source_table='white_side_table#living_room');h=[]
        r=dict(status='ok',room='living_room',arrived=True,source_table=s.task.source_table,target='dog_doll',confirmed=True,evidence_ref='synthetic',grasp_verified=True,holding_verified=True,carry_ready=True,destination=s.task.destination,release_sent=True,placement_verified=True)
        p={stage:Port(dict(r),h) for stage in s.steps};e=Executor(s,p,Port(dict(status='sent'),h))
        s.preflight(ready=True);e.moderator('Environment','LayoutA',session_id='test');e.moderator('Are_you_ready?',session_id='test')
        e.tick();e.tick();e.moderator('Instruction',INSTRUCTION,session_id='test')
        return s,e,p,h
    def test_chain(self):
        s,e,p,h=self.flow()
        for _ in range(60):e.tick()
        self.assertEqual(s.state,'waiting_moderator')
        self.assertEqual([x for x in h if x in s.steps],list(s.steps))
        self.assertLess(h.index('Object_grasped'),h.index('tuck'))
        self.assertLess(h.index('verify_placement'),h.index('Task_finished'))
        e.moderator('Task_succeeded',session_id='test');self.assertEqual(s.state,'succeeded')
    def test_cancel(self):
        s,e,p,h=self.flow();p['navigate_room'].pending=True;e.tick()
        e.moderator('Task_failed',session_id='test');e.tick()
        self.assertIn(('cancel',True),h);self.assertFalse(e.retired)
        p['navigate_room'].pending=False;e.tick();self.assertTrue(e.retired)
        self.assertNotIn('navigate_source',h)
    def test_publish_failure(self):
        s,e,p,h=self.flow();e.protocol.result={'status':'failed'}
        for _ in range(15):e.tick()
        self.assertEqual(s.state,'stopped');self.assertNotIn('navigate_source',h)
    def test_missing_ports(self):
        with self.assertRaises(ValueError):Executor(Session1('test'),{},Port({},[]))
    def test_failed_grasp(self):
        s,e,p,h=self.flow();p['grasp'].result={'status':'failed','reason':'not_held'}
        for _ in range(40):e.tick()
        self.assertEqual(s.reason,'not_held');self.assertNotIn('tuck',h)

import time
import unittest
from concurrent.futures import Future
from threading import Event
from application import Application
from session1 import Session1,INSTRUCTION

def done(value):f=Future();f.set_result(value);return f

class ApplicationTests(unittest.TestCase):
    def build(self,block=None):
        self.log=[];self.started=Event();self.stopped=[]
        fields=dict(status='ok',room='living_room',arrived=True,source_table='white_side_table#living_room',
                    target='dog_doll',confirmed=True,evidence_ref='synthetic',grasp_verified=True,
                    holding_verified=True,carry_ready=True,destination='square_low_table#living_room',release_sent=True,placement_verified=True)
        def handler(effect,cancel):
            self.log.append(effect['stage']);self.started.set()
            if block and effect['stage']=='navigate_room':cancel.wait(2)
            return {'status':'cancelled'} if cancel.is_set() else dict(fields)
        def stop(effect,**kw):self.stopped.append(kw);return done({'motion_stopped':True})
        steps=Session1('x',source_table='white_side_table#living_room').steps
        app=Application('x',{s:handler for s in steps},lambda e:done({'status':'sent'}),stop)
        app.session.preflight(ready=True)
        app.receive('Environment','LayoutA',session_id='x');app.receive('Are_you_ready?',session_id='x')
        app.tick();app.tick();app.receive('Instruction',INSTRUCTION,session_id='x')
        return app
    def until(self,app,predicate):
        end=time.monotonic()+3
        while not predicate() and time.monotonic()<end:app.tick();time.sleep(.001)
        self.assertTrue(predicate())
    def test_full_worker_chain(self):
        app=self.build();self.until(app,lambda:app.session.state=='waiting_moderator' and app.executor.retired)
        self.assertEqual(self.log,list(app.session.steps))
        app.receive('Task_succeeded',session_id='x');app.tick()
        self.assertEqual(app.session.state,'succeeded');app.close()
    def test_moderator_interrupts_running_handler(self):
        app=self.build(block=True);app.tick();self.assertTrue(self.started.wait(1))
        app.receive('Task_failed',session_id='x');app.tick()
        self.until(app,lambda:app.executor.retired)
        self.assertEqual(self.log,['navigate_room']);self.assertTrue(self.stopped[0]['preserve_grip']);app.close()
    def test_missing_real_handlers_rejected(self):
        with self.assertRaises(ValueError):Application('x',{},None,None)

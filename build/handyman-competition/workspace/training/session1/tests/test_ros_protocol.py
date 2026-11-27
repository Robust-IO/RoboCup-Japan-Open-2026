import unittest
from types import SimpleNamespace
from concurrent.futures import Future
from ros_protocol import RosProtocol,bind_application,TO_ROBOT,TO_MODERATOR
class Message:
    def __init__(self):self.message='';self.detail=''
class Node:
    def __init__(self):self.sent=[];self.topics=[];self.destroyed=[]
    def create_publisher(self,t,topic,q):self.topics.append(topic);return SimpleNamespace(publish=self.sent.append)
    def create_subscription(self,t,topic,cb,q):self.topics.append(topic);self.callback=cb;return 'sub'
    def create_timer(self,period,cb):self.tick=cb;return 'timer'
    def destroy_timer(self,t):self.destroyed.append(t)
    def destroy_subscription(self,s):self.destroyed.append(s)
    def destroy_publisher(self,p):self.destroyed.append('pub')
    def get_logger(self):return SimpleNamespace(warning=lambda s:None,info=lambda s:None)
class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.node=Node();self.messages=[]
        self.app=SimpleNamespace(session=SimpleNamespace(session_id='s',run_id='r'),tick=lambda:None,
            receive=lambda *a,**kw:self.messages.append((a,kw)),executor=SimpleNamespace(retired=True))
        self.p=RosProtocol(self.node,'s',message_type=Message);self.p.attach(self.app)
    def test_exact_topics_and_message_fields(self):
        self.assertEqual(self.node.topics,[TO_MODERATOR,TO_ROBOT])
        m=Message();m.message='Instruction';m.detail='task';self.node.callback(m)
        self.assertEqual(self.messages[0],(('Instruction','task'),{'session_id':'s'}))
    def test_publish_does_not_claim_scoring(self):
        f=self.p.publish(dict(session_id='s',run_id='r',message='Task_finished',detail=''))
        self.assertEqual(f.result()['status'],'sent');self.assertFalse(f.result()['moderator_acknowledged'])
        self.assertEqual(self.node.sent[0].message,'Task_finished')
    def test_stale_run_cannot_publish(self):
        with self.assertRaises(ValueError):self.p.publish(dict(session_id='s',run_id='old',message='Task_finished')).result()
        self.assertFalse(self.node.sent)
    def test_close_requires_action_retirement(self):
        self.app.executor.retired=False
        with self.assertRaises(RuntimeError):self.p.close()
        self.app.executor.retired=True;self.p.close();self.assertIn('pub',self.node.destroyed)
    def test_missing_stage_handlers_cleans_publisher(self):
        node=Node()
        with self.assertRaises(ValueError):bind_application(node,'s',{},None,message_type=Message)
        self.assertIn('pub',node.destroyed)

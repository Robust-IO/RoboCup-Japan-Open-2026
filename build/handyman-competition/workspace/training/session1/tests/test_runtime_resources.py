import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from runtime_resources import RuntimeResources
class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.events=[]
        (self.root/'semantic.json').write_text(json.dumps({'destinations':[{'id':'white_side_table#living_room'}]}))
        (self.root/'environment.yaml').write_text('{}');(self.root/'hand.json').write_text('{}')
        cfg=dict(layout='LayoutA',source_table_id='white_side_table#living_room',destination_id='square_low_table#living_room',task={'object':'dog_doll'},semantic_map=str(self.root/'semantic.json'),environment=str(self.root/'environment.yaml'),model_python='fake',registry='fake')
        (self.root/'config.json').write_text(json.dumps(cfg))
    def deps(self,fail_worker=False):
        events=self.events
        class Lock:
            def __enter__(self):events.append('lock');return self
            def __exit__(self,*args):events.append('unlock')
        def worker(*args,**kw):
            if fail_worker:raise RuntimeError('worker failed')
            return SimpleNamespace(close=lambda:events.append('worker-close'))
        return SimpleNamespace(ros=SimpleNamespace(ok=lambda:False,init=lambda:events.append('ros-init'),shutdown=lambda:events.append('ros-close'),create_node=lambda name:SimpleNamespace(destroy_node=lambda:events.append('node-close'))),
            yaml=SimpleNamespace(safe_load=lambda text:{}),lock=Lock,worker=worker,
            runtime_class=lambda:lambda *args:SimpleNamespace(shutdown=lambda:events.append('runtime-close')),
            physical=lambda:SimpleNamespace(hand=object()),motion=lambda cfg:SimpleNamespace(bind=lambda *args:None),
            grasp=lambda *args,**kw:SimpleNamespace(),hand=lambda data:object(),policy=lambda **kw:object())
    def owner(self,fail=False):return RuntimeResources(self.root/'config.json',self.root/'hand.json',self.root/'output',dependency_factory=lambda:self.deps(fail))
    def test_open_close_order(self):
        owner=self.owner()
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'71','ROS_LOCALHOST_ONLY':'1'}),patch.object(Path,'resolve',return_value=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/test')):
            owner.open();owner.close()
        self.assertEqual(self.events,['lock','ros-init','node-close','worker-close','runtime-close','ros-close','unlock'])
    def test_partial_startup_failure_cleans_owned_resources(self):
        owner=self.owner(True)
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'71','ROS_LOCALHOST_ONLY':'1'}),patch.object(Path,'resolve',return_value=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/test')):
            with self.assertRaises(RuntimeError):owner.open()
        self.assertEqual(self.events,['lock','ros-init','runtime-close','ros-close','unlock'])
        self.assertIsNone(owner.bundle)
    def test_active_application_keeps_lock(self):
        owner=self.owner()
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'71','ROS_LOCALHOST_ONLY':'1'}),patch.object(Path,'resolve',return_value=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/test')):
            owner.open()
            with self.assertRaises(RuntimeError):owner.close(SimpleNamespace(executor=SimpleNamespace(retired=False)))
            self.assertNotIn('unlock',self.events);owner.close()
    def test_no_implicit_environment_change(self):
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'0'}):
            with self.assertRaises(ValueError):self.owner().open()
        self.assertFalse(self.events)

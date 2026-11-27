import tempfile
import time
import unittest
from pathlib import Path
from concurrent.futures import Future,ThreadPoolExecutor
from types import SimpleNamespace
from application import WorkerPort
from motion_stop import StopController,MotionGate
class StopIntegrationTests(unittest.TestCase):
    def test_failed_handler_keeps_ownership_until_stop_proof(self):
        stop=Future();calls=[]
        def request(e,**kw):calls.append(kw);return stop
        with ThreadPoolExecutor(max_workers=1) as pool:
            port=WorkerPort(pool,lambda e,c:{'status':'failed','reason':'endpoint_missing'},request)
            h=port.start({'stage':'grasp'});h['future'].result(timeout=1)
            self.assertIsNone(port.poll(h));self.assertEqual(len(calls),1)
            self.assertIsNone(port.poll(h));stop.set_result({'motion_stopped':True})
            self.assertEqual(port.poll(h)['reason'],'endpoint_missing')
    def test_invalid_return_also_requires_stop(self):
        stop=Future();stop.set_result({'motion_stopped':True})
        with ThreadPoolExecutor(max_workers=1) as pool:
            port=WorkerPort(pool,lambda e,c:None,lambda *a,**kw:stop)
            h=port.start({'stage':'grasp'});h['future'].result(timeout=1)
            self.assertIsNone(port.poll(h));self.assertEqual(port.poll(h)['reason'],'invalid_stage_result')
    def test_real_stop_controller_thread_handoff(self):
        with tempfile.TemporaryDirectory() as root,ThreadPoolExecutor(max_workers=1) as pool:
            stamp=[0]
            def sample(**kw):stamp[0]+=200_000_000
            rt=SimpleNamespace(sample=sample,state=lambda:dict(stamp_ns=stamp[0],joints=[0,0,0,0,0,.3],poses={'odom':[0,0,0]}),pump=lambda t:sample())
            controller=StopController(rt,SimpleNamespace(active_child=None),MotionGate(),Path(root))
            action=pool.submit(lambda:{'status':'failed'});action.result(timeout=1)
            effect={'stage':'grasp','operation_id':'r/4'}
            app=SimpleNamespace(pool=pool,executor=SimpleNamespace(active=(None,{'future':action},dict(kind='operation',**effect))))
            controller.bind(app)
            result=controller.request_stop(effect,preserve_grip=True).result(timeout=2)
            self.assertTrue(result['motion_stopped']);self.assertTrue((Path(root)/'stop-confirmation.json').exists())
            controller.close()

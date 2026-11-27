"""Real local RPC and separate task clients, without loading GPU dependencies."""
import hashlib
import json
from pathlib import Path
import socketserver
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch

from apple_shared_inference import make_handler,registry_hash,SharedWorker
from apple_worker_client import Worker
from resident_inference import connect,service_key


class ResidentTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.root=Path(self.directory.name)
        self.weights=self.root/'best.pt';self.weights.write_bytes(b'unique test weights '+str(self.root).encode())
        self.registry=self.root/'model.json'
        self.registry.write_text(json.dumps(dict(weights=str(self.weights),sha256=hashlib.sha256(self.weights.read_bytes()).hexdigest(),imgsz=640,device=0)))
        self.servers=[]

    def tearDown(self):
        for server,thread in self.servers:
            endpoint=Path(server.server_address)
            server.shutdown();thread.join(2);server.server_close();endpoint.unlink(missing_ok=True)
        self.directory.cleanup()

    def start_fake(self,command,**kwargs):
        endpoint=command[command.index('--socket')+1]
        self.worker=SimpleNamespace(ready=dict(status='ready',imgsz=640,near_grasp_roi_enabled=False,
            model_sha256=json.loads(self.registry.read_text())['sha256'],model_pid=12345),
            infer=Mock(side_effect=lambda capture,**kw:dict(capture=str(capture),motion_authorized=False)))
        server=socketserver.UnixStreamServer(endpoint,make_handler(self.worker,self.root,registry_hash(self.registry),self.registry))
        thread=threading.Thread(target=server.serve_forever,kwargs=dict(poll_interval=.01));thread.start()
        self.servers.append((server,thread))
        return SimpleNamespace(pid=98765,poll=lambda:None)

    def test_separate_tasks_reuse_one_process_and_each_request_reads_its_capture(self):
        with patch('resident_inference.CAPTURE_ROOT',self.root),patch('resident_inference.subprocess.Popen',side_effect=self.start_fake) as launch:
            first=connect('/usr/bin/python3',self.registry,imgsz=640)
            a=first.infer(self.root/'task1/frame1');first.close()
            second=connect('/usr/bin/python3',self.registry,imgsz=640)
            b=second.infer(self.root/'task2/frame1');second.close()
            self.assertEqual(launch.call_count,1)
            self.assertFalse(first.ready['resident_reused']);self.assertTrue(second.ready['resident_reused'])
            self.assertEqual(first.ready['model_pid'],second.ready['model_pid'])
            self.assertNotEqual(a['capture'],b['capture']);self.assertEqual(self.worker.infer.call_count,2)
            self.assertFalse(a['motion_authorized'])

    def test_changed_registry_rejects_an_already_attached_client_without_inference(self):
        with patch('resident_inference.CAPTURE_ROOT',self.root),patch('resident_inference.subprocess.Popen',side_effect=self.start_fake):
            client=connect('/usr/bin/python3',self.registry,imgsz=640)
            self.registry.write_text(self.registry.read_text()+' ')
            with self.assertRaisesRegex(RuntimeError,'registry_changed'):client.infer(self.root/'frame')
            self.worker.infer.assert_not_called()

    def test_tampered_weights_never_attach(self):
        self.weights.write_bytes(b'other model')
        with self.assertRaisesRegex(ValueError,'model_hash_mismatch'):connect('/python',self.registry,imgsz=640)

    def test_resolution_and_roi_use_distinct_model_contexts(self):
        keys={service_key('/python',self.registry,size,roi) for size,roi in ((640,False),(960,False),(960,True))}
        self.assertEqual(len(keys),3)

    def test_worker_adapter_close_does_not_terminate_resident(self):
        client=Mock(ready=dict(status='ready',model_pid=123))
        with patch.dict('os.environ',{'HANDYMAN_RESIDENT_MODEL':'1'}),patch('resident_inference.connect',return_value=client),patch('apple_worker_client.subprocess.Popen') as process:
            worker=Worker('/python',str(self.registry),self.root/'task.log',imgsz=640)
            worker.infer(self.root/'fresh');worker.close()
            process.assert_not_called();client.infer.assert_called_once();client.close.assert_called_once()


if __name__=='__main__':unittest.main()

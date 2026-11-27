"""Cross-task, local-only model lifetime. Data/logs stay on D; IPC stays in /tmp.

The existing inference worker and shared protocol remain authoritative. No result
cache, image replay, robot interfaces or model changes. Closing a client leaves
the model running. Python/model/pipeline/resolution/ROI changes select a new key.
"""
import argparse
import fcntl
import hashlib
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import time
import uuid

from apple_shared_inference import SharedWorker,rpc

CAPTURE_ROOT=Path('/mnt/d/handyman-training-runs')


def service_key(python,registry,imgsz,roi):
    registry=Path(registry)
    value=json.loads(registry.read_text())
    if type(roi) is not bool or imgsz not in (640,960,1280):raise ValueError('invalid_resident_options')
    digest=hashlib.sha256(Path(value['weights']).read_bytes()).hexdigest()
    if digest!=value['sha256']:raise ValueError('model_hash_mismatch')
    code=Path(__file__).parent
    sources=('apple_inference_worker.py','apple_shared_inference.py','apple_worker_client.py',
             'resident_inference.py','apple_rgbd_candidate.py','apple_support_surface.py','apple_roi_confirmation.py')
    context=dict(python=str(Path(python).resolve()),registry=value,registry_path=str(registry.resolve()),
                 registry_bytes=hashlib.sha256(registry.read_bytes()).hexdigest(),imgsz=imgsz,roi=roi,
                 code={n:hashlib.sha256((code/n).read_bytes()).hexdigest() for n in sources})
    return hashlib.sha256(json.dumps(context,sort_keys=True).encode()).hexdigest()[:24]


def connect(python,registry,*,imgsz=None,confirm_apple_roi=False,timeout=90.):
    registry=Path(registry).resolve()
    if imgsz is None:imgsz=json.loads(registry.read_text())['imgsz']
    key=service_key(python,registry,imgsz,confirm_apple_roi)
    directory=Path('/tmp')/('hm-resident-'+str(os.getuid()))
    directory.mkdir(mode=0o700,exist_ok=True)
    if directory.stat().st_uid!=os.getuid() or directory.stat().st_mode&0o077:
        raise ValueError('resident_socket_directory_not_private')
    endpoint=directory/(key+'.sock')
    started=time.monotonic();created=False
    with (directory/(key+'.lock')).open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        try:
            rpc(endpoint,dict(op='ready',request_id=uuid.uuid4().hex),timeout=2.)
        except (FileNotFoundError,ConnectionRefusedError):
            endpoint.unlink(missing_ok=True)
            run=CAPTURE_ROOT/'resident-model'/ (key+'-'+uuid.uuid4().hex[:8])
            run.mkdir(parents=True,exist_ok=False)
            command=[sys.executable,str(Path(__file__).with_name('apple_shared_inference.py')),
                '--socket',str(endpoint),'--run-root',str(CAPTURE_ROOT),
                '--python',str(python),'--registry',str(registry),'--imgsz',str(imgsz),
                '--worker-log',str(run/'model.log')]
            if confirm_apple_roi:command.append('--confirm-apple-roi')
            environment=os.environ.copy();environment.pop('PYTHONPATH',None)
            with (run/'service.log').open('xb') as log:
                child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,
                    stderr=subprocess.STDOUT,env=environment,start_new_session=True)
            receipt=dict(pid=child.pid,key=key,endpoint=str(endpoint),command=command,started_ns=time.time_ns())
            (run/'launch.json').write_text(json.dumps(receipt,indent=2))
            # Poll readiness, not inference: no request or physical action is replayed.
            while True:
                if child.poll() is not None:raise RuntimeError('resident_model_startup_failed:'+str(run/'service.log'))
                try:
                    rpc(endpoint,dict(op='ready',request_id=uuid.uuid4().hex),timeout=2.);break
                except (FileNotFoundError,ConnectionRefusedError):
                    if time.monotonic()-started>timeout:
                        os.killpg(child.pid,signal.SIGTERM)
                        raise TimeoutError('resident_model_startup_timeout')
                    time.sleep(.05)
            created=True
        client=SharedWorker(endpoint,registry,imgsz=imgsz,confirm_apple_roi=confirm_apple_roi,timeout=2.)
        client.ready.update(resident_model=True,resident_reused=not created,resident_key=key,
                            attach_s=time.monotonic()-started)
        return client


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--python',required=True);p.add_argument('--registry',required=True)
    p.add_argument('--imgsz',type=int,choices=(640,960,1280),default=640)
    p.add_argument('--confirm-apple-roi',action='store_true')
    a=p.parse_args()
    client=connect(a.python,a.registry,imgsz=a.imgsz,confirm_apple_roi=a.confirm_apple_roi)
    print(json.dumps(client.ready));client.close()


if __name__=='__main__':main()

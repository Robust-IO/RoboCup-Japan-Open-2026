"""Bounded subprocess transport, independent of ROS and model dependencies."""
import json
import os
from pathlib import Path
import selectors
import subprocess
import time


def freshness(stamp_ns, now_ns, received_mono, checked_mono, max_age_s=.5):
    age=(now_ns-stamp_ns)/1e9
    elapsed=checked_mono-received_mono
    return dict(capture_to_result_age_s=age,request_roundtrip_s=elapsed,
                freshness_passed=0<=age<=max_age_s and 0<=elapsed<=max_age_s,
                motion_authorized=False)


class Worker:
    def __init__(self,python,registry,log,imgsz=None,*,confirm_apple_roi=False):
        if imgsz is not None and imgsz not in (640,960,1280):raise ValueError('unsupported_resolution')
        if type(confirm_apple_roi) is not bool:raise ValueError('invalid_roi_option')
        self.resident=None
        if os.environ.get('HANDYMAN_RESIDENT_MODEL')=='1':
            from resident_inference import connect
            self.resident=connect(python,registry,imgsz=imgsz,confirm_apple_roi=confirm_apple_roi)
            self.ready=self.resident.ready
            Path(log).write_text(json.dumps(self.ready,indent=2)+'\n')
            return
        self.log=Path(log).open('x');env=os.environ.copy();env.pop('PYTHONPATH',None)
        command=[python,str(Path(__file__).with_name('apple_inference_worker.py')),'--registry',registry]
        if imgsz is not None:command+=['--imgsz',str(imgsz)]
        if confirm_apple_roi:command+=['--confirm-apple-roi']
        self.p=subprocess.Popen(command,
                                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1,env=env)
        self.selector=selectors.DefaultSelector();self.selector.register(self.p.stdout,selectors.EVENT_READ)
        try:
            self.ready=self.read(90)
            if self.ready.get('status')!='ready':raise RuntimeError('worker_not_ready')
        except BaseException:self.close();raise

    def read(self,timeout,pump=None):
        deadline=time.monotonic()+timeout
        while True:
            remaining=deadline-time.monotonic()
            if remaining<=0:raise TimeoutError('worker_response_timeout')
            if self.selector.select(min(.01,remaining) if pump else remaining):break
            if pump:pump()
            elif time.monotonic()>=deadline:raise TimeoutError('worker_response_timeout')
        line=self.p.stdout.readline()
        if not line:raise RuntimeError('worker_exited')
        return json.loads(line)

    def infer(self,capture,pump=None,*,confirm_apple_roi=None):
        if getattr(self,'resident',None) is not None:
            return self.resident.infer(capture,pump=pump,confirm_apple_roi=confirm_apple_roi)
        request_id=str(time.monotonic_ns())
        request=dict(request_id=request_id,capture=str(capture))
        if confirm_apple_roi is not None:request['confirm_apple_roi']=confirm_apple_roi
        self.p.stdin.write(json.dumps(request)+'\n');self.p.stdin.flush()
        result=self.read(5,pump=pump)
        if result.get('request_id')!=request_id:raise RuntimeError('worker_request_mismatch')
        return result

    def close(self):
        if getattr(self,'resident',None) is not None:
            self.resident.close();return
        if self.p.poll() is None:
            self.p.stdin.close()
            try:self.p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.p.terminate()
                try:self.p.wait(timeout=2)
                except subprocess.TimeoutExpired:self.p.kill();self.p.wait(timeout=2)
        self.selector.close();self.p.stdout.close();self.log.close()

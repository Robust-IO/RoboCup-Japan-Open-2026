"""Task-owned Unix-socket model reuse. Read-only; never publishes robot commands.

Only initial connection readiness can wait/reconnect. An inference request is
sent once: an ambiguous response stops the caller, never silently replays it.
The endpoint lives in a private temporary directory, not on the network.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import selectors
import signal
import socket
import socketserver
import time
import uuid


def registry_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rpc(endpoint, request, *, timeout=5., pump=None):
    deadline = time.monotonic() + timeout
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(min(1., timeout))
        connection.connect(str(endpoint))
        connection.sendall((json.dumps(request, allow_nan=False) + '\n').encode())
        connection.setblocking(False)
        received = bytearray()
        with selectors.DefaultSelector() as selector:
            selector.register(connection, selectors.EVENT_READ)
            while time.monotonic() < deadline:
                if pump:
                    pump()
                if not selector.select(min(.01, max(0., deadline - time.monotonic()))):
                    continue
                block = connection.recv(65536)
                if not block:
                    raise RuntimeError('shared_worker_disconnected')
                received.extend(block)
                if len(received) > 8_000_000:
                    raise RuntimeError('shared_worker_response_too_large')
                if b'\n' in received:
                    response = json.loads(received.split(b'\n', 1)[0])
                    if response.get('request_id') != request['request_id']:
                        raise RuntimeError('shared_worker_request_mismatch')
                    if 'service_error' in response:
                        raise RuntimeError('shared_worker_error:' + response['service_error'])
                    return response
        raise TimeoutError('shared_worker_response_timeout')


class SharedWorker:
    def __init__(self, endpoint, registry, *, imgsz=960, confirm_apple_roi=False, timeout=90.):
        self.endpoint = endpoint
        self.roi = confirm_apple_roi
        deadline = time.monotonic() + timeout
        while True:
            try:
                reply = rpc(endpoint, dict(op='ready', request_id=uuid.uuid4().hex), timeout=2.)
                break
            except (FileNotFoundError, ConnectionRefusedError):
                if time.monotonic() >= deadline:
                    raise RuntimeError('shared_worker_startup_timeout') from None
                time.sleep(.02)
        expected = registry_hash(registry)
        model_sha = json.loads(Path(registry).read_text())['sha256']
        ready = reply['ready']
        if (reply.get('registry_sha256') != expected or ready.get('status') != 'ready'
                or ready.get('model_sha256') != model_sha
                or ready.get('imgsz') != imgsz
                or confirm_apple_roi and not ready.get('near_grasp_roi_enabled')):
            raise ValueError('shared_worker_context_mismatch')
        self.ready = dict(ready, shared_model=True)

    def infer(self, capture, pump=None, *, confirm_apple_roi=None):
        roi=self.roi if confirm_apple_roi is None else confirm_apple_roi
        if type(roi) is not bool:raise ValueError('unsupported_shared_roi_request')
        reply = rpc(self.endpoint, dict(op='infer', request_id=uuid.uuid4().hex,
                    capture=str(Path(capture).resolve()), confirm_apple_roi=roi), pump=pump)
        return reply['result']

    def close(self):
        # Caller owns only a connection, not the model or service lifetime.
        pass


def make_handler(worker, run_root, registry_sha256, registry_path=None):
    root = Path(run_root).resolve()

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.connection.settimeout(5.)
            request = {}
            try:
                line = self.rfile.readline(65537)
                if len(line) > 65536 or not line.endswith(b'\n'):
                    raise ValueError('invalid_shared_request')
                request = json.loads(line)
                if registry_path is not None and registry_hash(registry_path)!=registry_sha256:
                    raise ValueError('model_registry_changed_restart_service')
                if getattr(worker,'p',None) is not None and worker.p.poll() is not None:
                    raise RuntimeError('model_worker_exited')
                if request.get('op') == 'ready':
                    response = dict(ready=worker.ready, registry_sha256=registry_sha256,service_pid=os.getpid())
                elif request.get('op') == 'infer':
                    capture = Path(request['capture']).resolve()
                    if not capture.is_relative_to(root):
                        raise ValueError('capture_outside_owned_transfer')
                    roi = request.get('confirm_apple_roi', False)
                    if type(roi) is not bool or roi and not worker.ready.get('near_grasp_roi_enabled'):
                        raise ValueError('unsupported_shared_roi_request')
                    response = dict(result=worker.infer(capture, confirm_apple_roi=roi))
                else:
                    raise ValueError('unknown_shared_operation')
            except Exception as exc:
                response = dict(service_error=str(exc))
            response['request_id'] = request.get('request_id')
            try:
                self.wfile.write((json.dumps(response, allow_nan=False) + '\n').encode())
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--python', required=True)
    parser.add_argument('--registry', required=True)
    parser.add_argument('--confirm-apple-roi', action='store_true')
    parser.add_argument('--imgsz',type=int,choices=(640,960,1280),default=960)
    parser.add_argument('--worker-log',type=Path)
    args = parser.parse_args()
    def terminate(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,terminate)
    from apple_worker_client import Worker
    # The service owns a real model subprocess; never recursively attach itself.
    os.environ.pop('HANDYMAN_RESIDENT_MODEL',None)
    signature = registry_hash(args.registry)
    worker = Worker(args.python, args.registry, args.worker_log or args.run_root/'shared-model.worker.log',
                    imgsz=args.imgsz, confirm_apple_roi=args.confirm_apple_roi)
    worker.ready=dict(worker.ready,model_pid=worker.p.pid)
    try:
        if registry_hash(args.registry) != signature:
            raise ValueError('model_registry_changed_during_warmup')
        with socketserver.UnixStreamServer(str(args.socket),
                make_handler(worker, args.run_root, signature,args.registry)) as service:
            os.chmod(args.socket, 0o600)
            print(json.dumps(dict(status='ready', model=worker.ready, robot_commands_sent=False)), flush=True)
            try:
                service.serve_forever(poll_interval=.05)
            except KeyboardInterrupt:
                pass
    finally:
        worker.close()


if __name__ == '__main__':
    main()

"""Independent cleanup steps: a logging failure must not skip process cleanup."""
import os
import signal
import subprocess

def cleanup_steps(steps):
    errors=[]
    for name,action in steps:
        try: action()
        except Exception as exc: errors.append((name,str(exc)))
    return errors

def stop_worker(worker):
    if worker is None: return
    # The caller creates a dedicated session; include Pixi's Python descendants.
    try: worker.wait(timeout=3)
    except subprocess.TimeoutExpired: pass
    for sig in (signal.SIGTERM,signal.SIGKILL):
        try: os.killpg(worker.pid,sig)
        except ProcessLookupError: break
        try: worker.wait(timeout=3)
        except subprocess.TimeoutExpired: continue
    worker.wait(timeout=3)

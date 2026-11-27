"""Model-only prewarm: no robot or ROS imports."""
import json,subprocess,sys
from pathlib import Path

def command(config,package):
    cfg=json.loads(Path(config).read_text())
    return [sys.executable,str(Path(package)/'tools/apple-transfer-auto-v1/resident_inference.py'),
            '--python',cfg['model_python'],'--registry',cfg['registry'],'--imgsz','960']

def start(config,package,output):
    with Path(output).open('x') as log:
        return subprocess.Popen(command(config,package),stdin=subprocess.DEVNULL,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

def stop_waiter(child):
    if child is not None and child.poll() is None:
        child.terminate()
        try:child.wait(timeout=3)
        except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)

"""Model I/O in a worker thread; ROS preparation stays on the owning thread."""
from concurrent.futures import ThreadPoolExecutor
import json,time
from startup_hand import prepare

def prepare_with_model(factory,runtime,output):
    with ThreadPoolExecutor(max_workers=1) as pool:
        future=pool.submit(factory)
        try:
            started=time.monotonic();report=prepare(runtime)
            report['elapsed_s']=time.monotonic()-started
            (output/'initial-hand-preparation.json').write_text(json.dumps(report))
        except BaseException:
            try:future.result().close()
            except Exception:pass
            raise
        return future.result()

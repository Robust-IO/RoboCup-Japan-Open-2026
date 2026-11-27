import sys,json,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from stopped_capture_runtime import with_stopped_capture
from horizontal_can_trial import HorizontalCanRuntime
import rclpy
root=b/('controller-hand-check-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir();rclpy.init();r=with_stopped_capture(HorizontalCanRuntime,profile='paired')(root,'controller-hand-check',lambda e:None)
try:
 item=r.sample();cap=r.save(item,'paired');print(json.dumps(dict(capture=str(cap),state=r.state())));(b/'controller-hand-check-latest.json').write_text(json.dumps(dict(capture=str(cap),output=str(root))))
finally:r.shutdown();rclpy.shutdown()

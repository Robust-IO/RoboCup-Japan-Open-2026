import json,sys,time,uuid
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
import rclpy
from pepper_runtime import PepperRuntime
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'pepper-alignment-current-state-20261026-01';root.mkdir(exist_ok=False)
result=dict(robot_commands_sent=False);runtime=None
rclpy.init()
try:
    runtime=PepperRuntime(root,'resident-readonly-'+uuid.uuid4().hex,lambda _:None)
    started=time.monotonic()
    while time.monotonic()-started<4:runtime.pump(.02)
    result['stream_ages_s']={k:(runtime.node.get_clock().now().nanoseconds-runtime.stamp(m))/1e9 for k,m in runtime.received.items() if hasattr(m,'header')}
    sample=runtime.sample(processing_reserve_ns=0);result['capture']=str(runtime.save(sample,'current-scene'));result['state']=runtime.state()
    result['status']='fresh_state_observed'
except Exception as e:result.update(status='live_state_unavailable',reason=str(e))
finally:
    if runtime:runtime.shutdown()
    rclpy.shutdown();(root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))

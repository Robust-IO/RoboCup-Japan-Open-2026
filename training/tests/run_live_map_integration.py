"""Publish decoded map on isolated test topic, then deliberately change a cell."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='73' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile,DurabilityPolicy
    root=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(root/'training/scripts'))
    from verify_live_map import decode
    map_path=root/'src/handyman_rebuild_ros2/maps/LayoutA/map.yaml'
    helper=Path('/tmp/handyman-map-snapshot-build/map_snapshot')
    expected,digest=decode(map_path,helper)
    out=Path(tempfile.mkdtemp(prefix='map-equality-test-'));print('OUTPUT',out,flush=True)
    rclpy.init();node=rclpy.create_node('synthetic_map_publisher')
    pub=node.create_publisher(OccupancyGrid,'/handyman_test/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    process=None
    try:
        for case in ('matching','changed_cell'):
            msg=copy.deepcopy(expected)
            if case=='changed_cell':msg.data[0]=100 if msg.data[0]!=100 else 0
            with (out/(case+'.console')).open('w') as log:
                process=subprocess.Popen([sys.executable,str(root/'training/scripts/verify_live_map.py'),
                    '--map-yaml',str(map_path),'--decoder',str(helper),'--seconds','3',
                    '--topic','/handyman_test/map','--output',str(out/(case+'.json'))],stdout=log,stderr=subprocess.STDOUT)
                end=time.monotonic()+10
                while process.poll() is None and time.monotonic()<end:
                    pub.publish(msg);rclpy.spin_once(node,timeout_sec=.1)
                assert process.poll()==0,'probe failed or timed out'
            report=json.loads((out/(case+'.json')).read_text())
            assert report['messages'] and report['consistent']==(case=='matching'),report
            if case=='changed_cell':assert all(m['reason']=='map_cells_mismatch' for m in report['messages'])
            print(case,'PASS',flush=True)
    finally:
        if process is not None and process.poll() is None:process.terminate();process.wait(timeout=5)
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()

"""Read-only /map equality check against Nav2-decoded local files, not costmap proof."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
import yaml


def compare(expected,actual):
    def reject(reason):return dict(matches=False,reason=reason,actionable=False)
    if actual.header.frame_id!='map':return reject('wrong_map_frame')
    a,b=expected.info,actual.info
    if (a.width,a.height)!=(b.width,b.height) or a.width<=0 or a.height<=0:
        return reject('map_dimensions_mismatch')
    def geometry(info):
        p,q=info.origin.position,info.origin.orientation
        return (info.resolution,p.x,p.y,p.z,q.x,q.y,q.z,q.w)
    # Both maps use ROS message representation, so no decimal conversion tolerance needed.
    av,bv=geometry(a),geometry(b)
    if not all(math.isfinite(v) for v in av+bv) or av!=bv:
        return reject('map_geometry_mismatch')
    if len(actual.data)!=a.width*a.height or len(expected.data)!=a.width*a.height:
        return reject('invalid_map_data_length')
    if any(v not in range(-1,101) for v in actual.data):return reject('invalid_occupancy_value')
    if list(expected.data)!=list(actual.data):return reject('map_cells_mismatch')
    return dict(matches=True,reason='received_map_matches_local_files',actionable=False,
        width=a.width,height=a.height,costmap_verified=False,
        cells_sha256=hashlib.sha256(bytes(v & 255 for v in actual.data)).hexdigest())


def bundle(map_path):
    content=map_path.read_bytes();meta=yaml.safe_load(content)
    image=(map_path.parent/meta['image']).resolve()
    if not image.is_relative_to(map_path.parent.resolve()):raise ValueError('image_outside_map_directory')
    return hashlib.sha256(content+b'\0'+image.read_bytes()).hexdigest()


def decode(map_path,helper):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import OccupancyGrid
    before=bundle(map_path)
    with tempfile.TemporaryDirectory(prefix='map-reference-') as temporary:
        path=Path(temporary)/'map.cdr'
        subprocess.run([str(helper),str(map_path),str(path)],check=True,timeout=15,
            stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        expected=deserialize_message(path.read_bytes(),OccupancyGrid)
    if bundle(map_path)!=before:raise ValueError('map_files_changed_while_decoding')
    return expected,before


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map-yaml',type=Path,required=True)
    p.add_argument('--decoder',type=Path,required=True)
    p.add_argument('--topic',default='/map')
    p.add_argument('--seconds',type=float,default=8)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not 0<a.seconds<=30:p.error('seconds must be in (0,30]')
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile,DurabilityPolicy
    expected,digest=decode(a.map_yaml.resolve(),a.decoder.resolve())
    with a.output.open('x') as stream:
        rclpy.init();node=rclpy.create_node('handyman_map_equality_readonly');rows=[]
        def receive(msg):
            result=compare(expected,msg)
            result['received_monotonic']=time.monotonic()
            rows.append(result)
        try:
            sub=node.create_subscription(OccupancyGrid,a.topic,receive,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            end=time.monotonic()+a.seconds
            while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
            unchanged=bundle(a.map_yaml.resolve())==digest
            count=len(node.get_publishers_info_by_topic(a.topic))
            report=dict(map_bundle_sha256=digest,messages=rows,local_files_unchanged=unchanged,
                publisher_count=count,consistent=bool(rows) and all(r['matches'] for r in rows) and unchanged and count==1,
                actionable=False,costmap_verified=False,scope='received static map only; no navigation readiness claim')
            json.dump(report,stream,indent=2,allow_nan=False);print(json.dumps(report),flush=True)
        finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()

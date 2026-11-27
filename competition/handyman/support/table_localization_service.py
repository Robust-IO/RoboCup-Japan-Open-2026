"""Single persistent map->odom owner; correction only on stopped requests."""
import argparse,json,threading,time
from collections import deque
from pathlib import Path
import numpy as np,yaml
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster
from table_localization import config,align,decide,stationary,compose,pose,angle,yaw

def td(message):
    t=message.transform;return dict(translation=[t.translation.x,t.translation.y,t.translation.z],quaternion=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
def write(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2));tmp.replace(path)

class TableLocalizer(Node):
    def __init__(self,args):
        super().__init__('handyman_table_localization');self.args=args;self.cfg=config();self.anchor=json.loads(args.initial_anchor.read_text());self.active=False;self.serial=0;self.lock=threading.Lock()
        self.buf=Buffer();self.listener=TransformListener(self.buf,self);self.queue=deque(maxlen=50);self.group=ReentrantCallbackGroup()
        self.sub=self.create_subscription(LaserScan,'/hsrb/base_scan',self.queue.append,qos_profile_sensor_data,callback_group=self.group)
        meta=yaml.safe_load(args.map.read_text());self.grid=np.asarray(Image.open(args.map.parent/meta['image']));self.origin=meta['origin'];self.resolution=meta['resolution']
        self.service=self.create_service(Trigger,'/handyman/table_localization/check',self.check,callback_group=self.group)
        self.timer=self.create_timer(.1,self.activate);write(args.output/'service-status.json',dict(status='waiting_for_owner_handoff',robot_commands_sent=False))
    def activate(self):
        if self.active or not self.args.activate_file.exists():return
        self.broadcaster=StaticTransformBroadcaster(self);self.publish(self.anchor);self.active=True
        write(self.args.output/'service-status.json',dict(status='active',robot_commands_sent=False,anchor=self.anchor))
    def publish(self,anchor):
        msg=TransformStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='map';msg.child_frame_id='odom'
        msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=anchor['translation']
        msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=anchor['quaternion']
        self.broadcaster.sendTransform(msg);self.anchor=anchor
    def fresh_odom(self):
        tf=self.buf.lookup_transform('odom','base_footprint',Time())
        if not -.02<=(self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9<=.6:raise ValueError('current_odometry_unavailable')
        return td(tf)
    def collect(self):
        rows=[];seen=set();start=self.get_clock().now().nanoseconds;deadline=time.monotonic()+8
        while time.monotonic()<deadline and len(rows)<self.cfg['scans']:
            for scan in list(self.queue):
                stamp=Time.from_msg(scan.header.stamp);ns=stamp.nanoseconds
                if ns<start or ns in seen or not 0<=(self.get_clock().now().nanoseconds-ns)/1e9<.4 or (rows and ns-rows[-1]['stamp_ns']<100_000_000):continue
                pairs=[('base_from_scan','base_footprint',scan.header.frame_id),('odom_from_base','odom','base_footprint'),('map_from_base','map','base_footprint')]
                if not all(self.buf.can_transform(t,s,stamp) for _,t,s in pairs):continue
                row=dict(stamp_ns=ns,angle_min=scan.angle_min,angle_increment=scan.angle_increment,ranges=[float(v) if np.isfinite(v) else None for v in scan.ranges])
                row.update({key:td(self.buf.lookup_transform(t,s,stamp)) for key,t,s in pairs});rows.append(row);seen.add(ns);break
            time.sleep(.01)
        if len(rows)!=self.cfg['scans']:raise ValueError('fresh_scan_time_transforms_unavailable')
        return rows
    def check(self,request,response):
        if not self.lock.acquire(blocking=False):response.success=False;response.message=json.dumps(dict(status='busy'));return response
        self.serial+=1;path=self.args.output/f'check-{self.serial:04d}.json';report=dict(status='checking',robot_commands_sent=False,correction_applied=False)
        try:
            if not self.active:raise ValueError('single_map_owner_not_active')
            if any(v in self.get_node_names() for v in ('initial_pose_localizer','reviewed_bedroom_map_anchor')):raise ValueError('competing_map_owner_detected')
            before=self.fresh_odom();rows=self.collect()
            for row in rows:
                if not stationary(before,row['odom_from_base'],self.cfg):raise ValueError('base_moved_during_localization')
            results=[align(row,self.grid,self.origin,self.resolution,self.cfg) for row in rows];proof=decide(results,self.cfg)
            if not stationary(before,self.fresh_odom(),self.cfg):raise ValueError('base_moved_during_localization')
            report.update(proof,original_anchor=self.anchor,map_resolution_m=self.resolution)
            if proof['apply_correction']:
                self.publish(proof['anchor']);report['correction_applied']=True
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    tf=td(self.buf.lookup_transform('map','odom',Time()))
                    if (np.linalg.norm(np.asarray(tf['translation'])-proof['anchor']['translation'])<.002
                        and abs(angle(yaw(tf['quaternion'])-yaw(proof['anchor']['quaternion'])))<.002):break
                    time.sleep(.02)
                else:raise ValueError('corrected_map_transform_not_received')
            report['status']='localization_verified';report['current_base_pose']=pose(compose(self.anchor,self.fresh_odom()));response.success=True
            # Store raw scans so every correction can be independently replayed.
            write(self.args.output/f'check-{self.serial:04d}-scans.json',rows)
        except Exception as exc:report.update(status='localization_not_verified',reason=str(exc));response.success=False
        finally:write(path,report);response.message=json.dumps(report);self.lock.release()
        return response

def main():
    p=argparse.ArgumentParser();p.add_argument('--initial-anchor',type=Path,required=True);p.add_argument('--activate-file',type=Path,required=True);p.add_argument('--map',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rclpy.init();node=TableLocalizer(a);executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()

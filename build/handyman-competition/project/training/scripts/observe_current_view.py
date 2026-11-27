"""Read-only online visual-search harness, not a navigation-arrival verifier.

Requires explicit operator-stationary assumption. Dummy search pose is never
executed. No publishers, action clients or competition replies are created.
"""
import argparse
import json
import math
from pathlib import Path
import time
import uuid
from search_scan import SearchScan
from search_vision_adapter import SearchVisionAdapter


class CurrentView:
    def __init__(self,target,started,sensor_start,seconds):
        self.scan=SearchScan(uuid.uuid4().hex,target,[dict(x=0,y=0,yaw=0)],started,
                             timeout_s=seconds,view_s=seconds)
        # Explicit harness assumption, NOT a fabricated successful Nav2 action.
        self.scan.arrived(self.scan.request()['view_id'],True,True,started)
        self.adapter=SearchVisionAdapter(self.scan,self.scan.request()['view_id'],sensor_start)

    def consume(self,row,now,sensor_now):
        return self.annotate(self.adapter.consume(row,now,sensor_now_ns=sensor_now))

    def annotate(self,state):
        return dict(state,scope='current_view_visual_search_harness',
            stationary_basis='operator_assumption_not_verified',navigation_arrival_verified=False,
            actionable=False,does_not_exist_authorized=False,
            position_m=list(self.scan.position) if self.scan.position else None,
            position_frame='odom',position_semantics='visible_surface_not_grasp_pose')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true')
    p.add_argument('--operator-stationary',action='store_true')
    p.add_argument('--target',default='canned_juice')
    p.add_argument('--seconds',type=float,default=30.)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if not math.isfinite(args.seconds) or not 0<args.seconds<=60:p.error('seconds must be (0,60]')
    if args.run and not args.operator_stationary:p.error('--run requires --operator-stationary')
    if not args.run:
        print('Preflight only; no ROS node or files. Unity may remain paused.');return
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String
    with args.output.open('x') as stream:
        rclpy.init();node=rclpy.create_node('handyman_current_view_readonly')
        started=time.monotonic()
        observer=CurrentView(args.target,started,node.get_clock().now().nanoseconds,args.seconds)
        def emit(row):
            stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
        def receive(msg):
            try:row=json.loads(msg.data)
            except (ValueError,TypeError):row={}
            if not isinstance(row,dict):row={}
            emit(observer.consume(row,time.monotonic(),node.get_clock().now().nanoseconds))
        sub=node.create_subscription(String,'/handyman/vision/diagnostics',receive,10)
        emit(observer.annotate(observer.scan.status()))
        print('READY: read-only current-view observer; no navigation arrival verified',flush=True)
        try:
            while rclpy.ok() and observer.scan.phase=='observe':
                rclpy.spin_once(node,timeout_sec=.05)
                observer.scan.tick(time.monotonic())
        except (KeyboardInterrupt,ExternalShutdownException):observer.scan.cancel()
        finally:
            result=observer.annotate(observer.scan.status());emit(dict(result,terminal=True))
            node.destroy_node();rclpy.try_shutdown()
            print(json.dumps(result),flush=True)


if __name__=='__main__':main()

"""Read-only post-stop feedback timing and map-pose audit. No publishers."""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState, LaserScan
    from tf2_ros import Buffer, TransformListener
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('output_exists')
    rclpy.init();n=rclpy.create_node('transfer_feedback_readonly_probe');b=Buffer();listener=TransformListener(b,n)
    received={};arrivals={'joints':[],'scan':[]};rows=[]
    def receive(key,m):
        received[key]=m;arrivals[key].append(dict(stamp_ns=Time.from_msg(m.header.stamp).nanoseconds,received_ns=time.time_ns()))
    subs=[n.create_subscription(JointState,'/hsrb/joint_states',lambda m:receive('joints',m),qos_profile_sensor_data),n.create_subscription(LaserScan,'/hsrb/base_scan',lambda m:receive('scan',m),qos_profile_sensor_data)]
    start=time.monotonic()
    try:
        while time.monotonic()-start<18:
            rclpy.spin_once(n,timeout_sec=.01)
            for _ in range(10):rclpy.spin_once(n,timeout_sec=0)
            if time.monotonic()-start<4 or len(received)!=2:continue
            row=dict(now_ns=n.get_clock().now().nanoseconds)
            for key,m in received.items():row[key+'_age_s']=(row['now_ns']-Time.from_msg(m.header.stamp).nanoseconds)/1e9
            try:
                tf=b.lookup_transform('map','base_footprint',Time());t=tf.transform.translation;q=tf.transform.rotation
                row['tf_age_s']=(row['now_ns']-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
                row['map_pose']=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            except Exception as exc:row['tf_error']=str(exc)
            rows.append(row)
    finally:
        result=dict(rows=rows,arrivals=arrivals,robot_commands_sent=False)
        if rows:
            result['last']=rows[-1]
            result['ages']={key:dict(min=min(r[key] for r in rows if key in r),max=max(r[key] for r in rows if key in r)) for key in ('joints_age_s','scan_age_s','tf_age_s') if any(key in r for r in rows)}
        result['max_receive_gap_s']={key:max(((v['received_ns']-u['received_ns'])/1e9 for u,v in zip(vals,vals[1:])),default=None) for key,vals in arrivals.items()}
        a.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('rows','arrivals')}));n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()

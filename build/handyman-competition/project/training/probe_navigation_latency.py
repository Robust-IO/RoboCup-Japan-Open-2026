"""Read-only navigation stream/TF arrival diagnostic. Never creates a publisher."""
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
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener

    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=30.)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not math.isfinite(a.seconds) or not 5<=a.seconds<=45 or a.output.exists():
        raise ValueError('invalid_readonly_diagnostic_request')
    rclpy.init();n=rclpy.create_node('navigation_latency_readonly_probe')
    b=Buffer();listener=TransformListener(b,n);data={};arrivals={};rows=[];pump=[]
    def receive(key,m):
        now=n.get_clock().now().nanoseconds
        stamp=Time.from_msg(m.header.stamp).nanoseconds
        arrivals.setdefault(key,[]).append(dict(received_ns=now,monotonic=time.monotonic(),stamp_ns=stamp,
            age_at_callback_s=(now-stamp)/1e9))
        if key in ('joints','scan'):data[key]=m
    def tf_receive(msg):
        for t in msg.transforms:receive(t.header.frame_id+'->'+t.child_frame_id,t)
    subs=[n.create_subscription(JointState,'/hsrb/joint_states',lambda m:receive('joints',m),qos_profile_sensor_data),
        n.create_subscription(LaserScan,'/hsrb/base_scan',lambda m:receive('scan',m),qos_profile_sensor_data),
        n.create_subscription(TFMessage,'/tf',tf_receive,qos_profile_sensor_data)]
    start=time.monotonic();next_sample=start+3
    try:
        while time.monotonic()-start<a.seconds:
            before=time.monotonic();rclpy.spin_once(n,timeout_sec=.01)
            for _ in range(20):rclpy.spin_once(n,timeout_sec=0)
            elapsed=time.monotonic()-before
            if elapsed>.03:pump.append(dict(elapsed_s=elapsed,at_s=time.monotonic()-start))
            if time.monotonic()<next_sample:continue
            next_sample=time.monotonic()+.05
            now=n.get_clock().now();r=dict(now_ns=now.nanoseconds,ages_s={})
            for key,m in data.items():r['ages_s'][key]=(now-Time.from_msg(m.header.stamp)).nanoseconds/1e9
            for parent,child in [('odom','base_footprint'),('map','base_footprint')]:
                try:
                    tf=b.lookup_transform(parent,child,Time())
                    r['ages_s'][parent+'->'+child]=(now-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
                    if parent=='map':
                        t=tf.transform.translation;q=tf.transform.rotation
                        r['map_pose']=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
                except Exception as exc:r[parent+'->'+child+'_error']=str(exc)
            rows.append(r)
    finally:
        summary={}
        for key,events in arrivals.items():
            gaps=[y['monotonic']-x['monotonic'] for x,y in zip(events,events[1:])]
            ages=[v['age_at_callback_s'] for v in events]
            summary[key]=dict(messages=len(events),max_receive_gap_s=max(gaps,default=None),
                gaps_above_0_7s=sum(v>.7 for v in gaps),age_at_callback_max_s=max(ages,default=None))
        sample_summary={}
        for key in ('joints','scan','odom->base_footprint','map->base_footprint'):
            ages=sorted(r['ages_s'][key] for r in rows if key in r['ages_s'])
            sample_summary[key]=dict(samples=len(ages),max_age_s=ages[-1] if ages else None,
                p95_age_s=ages[int(.95*(len(ages)-1))] if ages else None,
                above_0_7s=sum(v>.7 for v in ages))
        result=dict(robot_commands_sent=False,callback_summary=summary,sample_summary=sample_summary,
            pump_delays=pump,rows=rows,arrivals=arrivals,
            caveat='Bridge TF/scan use receipt timestamps; callback age and interarrival gaps do not prove Unity source timestamp alignment.')
        with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
        print(json.dumps(dict(output=str(a.output),callback_summary={k:v for k,v in summary.items()
            if k in ('joints','scan','odom->base_footprint','base_footprint->base_link')},
            sample_summary=sample_summary,pump_delays=pump,robot_commands_sent=False)),flush=True)
        n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()

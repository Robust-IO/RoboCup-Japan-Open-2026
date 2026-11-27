"""Read-only sampled TF-age audit. No changes to control thresholds."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

PAIRS=[('map','odom'),('odom','base_footprint'),('map','base_footprint'),('base_link','hand_palm_link')]


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    rclpy.init();n=rclpy.create_node('grasp_tf_timing_probe');b=Buffer();listener=TransformListener(b,n)
    arrivals={'tf':[], 'joints':[]}
    def record(key, stamps):
        now=n.get_clock().now().nanoseconds
        arrivals[key].append(dict(received=time.monotonic(),ages=[(now-Time.from_msg(s).nanoseconds)/1e9 for s in stamps]))
    subscriptions=[n.create_subscription(TFMessage,'/tf',lambda m:record('tf',[t.header.stamp for t in m.transforms]),qos_profile_sensor_data),
                   n.create_subscription(JointState,'/hsrb/joint_states',lambda m:record('joints',[m.header.stamp]),qos_profile_sensor_data)]
    rows=[];start=time.monotonic();last=0
    while time.monotonic()-start<15:
        rclpy.spin_once(n,timeout_sec=.01)
        elapsed=time.monotonic()-start
        if elapsed<2 or elapsed-last<.05:continue
        last=elapsed
        for parent,child in PAIRS:
            r=dict(elapsed=elapsed,parent=parent,child=child)
            try:
                tf=b.lookup_transform(parent,child,Time());r.update(age_s=(n.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9,
                    stamp_ns=Time.from_msg(tf.header.stamp).nanoseconds,xyz=[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z])
            except Exception as e:r['error']=str(e)
            rows.append(r)
    summary={}
    for parent,child in PAIRS:
        selected=[r for r in rows if r['parent']==parent and r['child']==child];valid=[r for r in selected if 'age_s' in r]
        ages=sorted(r['age_s'] for r in valid)
        summary[parent+'->'+child]=dict(samples=len(selected),errors=len(selected)-len(valid),distinct_stamps=len({r['stamp_ns'] for r in valid}),
            age_min=ages[0] if ages else None,age_max=ages[-1] if ages else None,age_median=ages[len(ages)//2] if ages else None,
            above_400ms=sum(v>.4 for v in ages))
    # Static transforms are timeless: zero stamp is not a stale dynamic sample.
    summary['map->odom']['static_timeless']=True
    for key, messages in arrivals.items():
        gaps=[y['received']-x['received'] for x,y in zip(messages,messages[1:])]
        ages=sorted(v for m in messages for v in m['ages'])
        summary[key+'_arrival']=dict(messages=len(messages),max_gap_s=max(gaps,default=0),gaps_above_400ms=sum(g>.4 for g in gaps),
            max_stamp_age_s=max(ages,default=0),median_stamp_age_s=ages[len(ages)//2] if ages else None)
    with a.output.open('x') as f:json.dump(dict(summary=summary,rows=rows,arrivals=arrivals),f,indent=2)
    print(json.dumps(summary));n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()

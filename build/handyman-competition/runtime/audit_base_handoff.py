"""Read-only verification after navigation RESET; no command publishers."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Output exists')
    rclpy.init();n=rclpy.create_node('base_handoff_readonly');b=Buffer();listener=TransformListener(b,n)
    names=['controller_server','planner_server','behavior_server','bt_navigator','map_server']
    clients={name:n.create_client(GetState,'/'+name+'/get_state') for name in names}
    futures={};deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.02)
        for name,c in clients.items():
            if name not in futures and c.service_is_ready():futures[name]=c.call_async(GetState.Request())
        if len(futures)==len(names) and all(f.done() for f in futures.values()):break
    # Allow discovery of publishers and fresh TF independently of service replies.
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
    states={name:f.result().current_state.label for name,f in futures.items() if f.done() and f.result()}
    publishers=[dict(name=e.node_name,namespace=e.node_namespace) for e in n.get_publishers_info_by_topic('/hsrb/command_velocity')]
    result=dict(read_only=True,states=states,velocity_publishers=publishers,
                velocity_subscribers=n.count_subscribers('/hsrb/command_velocity'),motion_authorized=False)
    try:
        tf=b.lookup_transform('map','base_footprint',Time());t=tf.transform.translation
        result['base_map_xyz']=[t.x,t.y,t.z]
        result['base_tf_age_s']=(n.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
    except Exception as e:result['tf_error']=str(e)
    result['handoff_verified']=(all(states.get(name)=='unconfigured' for name in names[:-1])
        and states.get('map_server')=='active' and not publishers and result['velocity_subscribers']==1
        and -.02<=result.get('base_tf_age_s',999)<=.4)
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result));n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()

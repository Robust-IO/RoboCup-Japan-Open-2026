"""Read-only selection diagnostics and actual post-stop state; no inference/motion."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from synced_capture_selection import select_stamps
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,JointState
from tf2_ros import Buffer,TransformListener

if os.environ.get('ROS_DOMAIN_ID')!='71':raise ValueError('domain71_required')
out=Path('/__HANDYMAN_PACKAGE__/runtime/apple-r12b-post-stop-20261018-01')
with RobotLock():
    out.mkdir(exist_ok=False)
    children={};logs={}
    commands=dict(
        cameras=[sys.executable,str(code/'capture_grasp_scene.py'),str(out/'cameras'),'--hand-geometry','--map-geometry',
            '--tracking-epoch',out.name,'--duration','7','--interval','1'],
        ready=[sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(out/'ready.json')])
    for name,cmd in commands.items():
        logs[name]=(out/(name+'.log')).open('x')
        children[name]=subprocess.Popen(cmd,stdout=logs[name],stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('r12b_pairing_readonly_diagnostic');buf=Buffer();listener=TransformListener(buf,node)
    received={};history={k:{} for k in ('rgb','depth','hand')};arrivals=[];rows=[]
    def stamp(msg):return Time.from_msg(msg.header.stamp).nanoseconds
    def on_message(key,msg):
        received[key]=msg
        arrivals.append(dict(stream=key,stamp_ns=stamp(msg),age_at_callback_s=(node.get_clock().now().nanoseconds-stamp(msg))/1e9))
        if key in history:
            history[key][stamp(msg)]=msg
            while len(history[key])>40:history[key].pop(next(iter(history[key])))
    topics=dict(rgb=('/hsrb/head_rgbd_sensor/rgb/image_raw',Image),
        depth=('/hsrb/head_rgbd_sensor/depth_registered/image_raw',Image),hand=('/hsrb/hand_camera/image_raw',Image),
        joints=('/hsrb/joint_states',JointState))
    subs=[node.create_subscription(kind,topic,lambda m,k=key:on_message(k,m),qos_profile_sensor_data) for key,(topic,kind) in topics.items()]
    def ready(stamps):
        rgb_ns,depth_ns,hand_ns=stamps;rgb=history['rgb'][rgb_ns];dep=history['depth'][depth_ns];hand=history['hand'][hand_ns]
        dt=Time(nanoseconds=depth_ns);ht=Time(nanoseconds=hand_ns)
        required=[('base_link',rgb.header.frame_id,dt),(rgb.header.frame_id,dep.header.frame_id,dt),
            ('odom','base_link',dt),('base_link','odom',ht),('map','base_link',dt),
            ('base_link',hand.header.frame_id,ht),('hand_palm_link',hand.header.frame_id,ht)]
        required +=[(hand.header.frame_id,link,ht) for link in
            ('hand_l_spring_proximal_link','hand_r_spring_proximal_link','hand_l_distal_link','hand_r_distal_link')]
        return all(buf.can_transform(t,s,when) for t,s,when in required)
    start=time.monotonic();last=0
    try:
        while time.monotonic()-start<12:
            rclpy.spin_once(node,timeout_sec=.01)
            for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
            elapsed=time.monotonic()-start
            if elapsed<2 or elapsed-last<.1 or len(received)!=4:continue
            last=elapsed;now=node.get_clock().now().nanoseconds;js=stamp(received['joints'])
            selections={str(reserve):select_stamps(history,now,ready=ready,joint_stamp_ns=js,processing_reserve_ns=reserve)
                        for reserve in (0,200_000_000)}
            rows.append(dict(now_ns=now,joint_age_s=(now-js)/1e9,
                latest_image_ages_s={k:(now-max(h))/1e9 for k,h in history.items()},selections=selections))
    finally:node.destroy_node();rclpy.shutdown()
    exits={name:child.wait(timeout=25) for name,child in children.items()}
    for f in logs.values():f.close()
    age_summary={}
    for stream in topics:
        ages=sorted(r['age_at_callback_s'] for r in arrivals if r['stream']==stream)
        age_summary[stream]=dict(messages=len(ages),minimum_s=min(ages,default=None),median_s=ages[len(ages)//2] if ages else None,
                                maximum_s=max(ages,default=None))
    result=dict(status='readonly_pairing_and_poststop_audit_complete',samples=len(rows),
        selection_counts={str(reserve):sum(r['selections'][str(reserve)] is not None for r in rows) for reserve in (0,200_000_000)},
        joint_age_min_s=min((r['joint_age_s'] for r in rows),default=None),joint_age_max_s=max((r['joint_age_s'] for r in rows),default=None),
        arrival_ages=age_summary,rows=rows,child_exit_codes=exits,
        readiness=json.loads((out/'ready.json').read_text()) if (out/'ready.json').exists() else None,
        last_capture=str(sorted((out/'cameras').glob('*/snapshot.json'))[-1].parent) if list((out/'cameras').glob('*/snapshot.json')) else None,
        robot_commands_sent=False,thresholds_changed=False)
    write_json(out/'result.json',result);print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)

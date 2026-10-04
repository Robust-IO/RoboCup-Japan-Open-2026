"""Isolated real-Nav2 search-turn test with synthetic TF and free/blocked costmap.

Never use domain 71. No bridge, camera, or real robot topics are used.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

def main():
    if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('isolated domain 73 required')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped, Twist, PolygonStamped, Point32
    from nav2_msgs.msg import Costmap
    from nav2_msgs.action import NavigateToPose
    from lifecycle_msgs.srv import ChangeState
    from tf2_ros import TransformBroadcaster
    from ament_index_python.packages import get_package_share_directory
    import yaml
    out=Path(tempfile.mkdtemp(prefix='search-turn-nav2-'))
    share=Path(get_package_share_directory('handyman_rebuild_ros2'))
    base=Path(get_package_share_directory('handyman_ros2'))
    params=yaml.safe_load((base/'param/nav2_params.yaml').read_text())
    bp=params['behavior_server']['ros__parameters']
    bp.update(behavior_plugins=['search_spin'],search_spin={'plugin':'handyman_rebuild_ros2/SearchSpin'},
              costmap_topic='/handyman_test/turn/costmap',footprint_topic='/handyman_test/turn/footprint',
              cycle_frequency=20.,transform_tolerance=.1)
    np=params['bt_navigator']['ros__parameters']
    np['plugin_lib_names'].append('handyman_search_turn_bt')
    np['default_nav_to_pose_bt_xml']=str(share/'behavior_trees/search_turn.xml')
    unused_tree=out/'unused-through.xml'
    unused_tree.write_text('<root main_tree_to_execute="Unused"><BehaviorTree ID="Unused"><AlwaysSuccess/></BehaviorTree></root>')
    np['default_nav_through_poses_bt_xml']=str(unused_tree)
    config=out/'params.yaml';config.write_text(yaml.safe_dump(params))
    rclpy.init();node=rclpy.create_node('search_turn_fixture')
    broadcaster=TransformBroadcaster(node)
    cm=node.create_publisher(Costmap,bp['costmap_topic'],QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    fp=node.create_publisher(PolygonStamped,bp['footprint_topic'],10)
    state=dict(yaw=2.30928391656,x=0.,y=0.,w=0.,blocked=False,publish_tf=True)
    commands=[]
    def velocity(msg):
        assert msg.linear.x==0 and msg.linear.y==0
        state['w']=msg.angular.z;commands.append((time.monotonic(),msg.angular.z))
    sub=node.create_subscription(Twist,'/handyman_test/turn/cmd_vel',velocity,10)
    last=time.monotonic();last_map=0.
    def tick():
        nonlocal last,last_map
        now=time.monotonic();dt=now-last;last=now
        state['yaw']+=state['w']*min(dt,.1)
        stamp=node.get_clock().now().to_msg()
        if state['publish_tf']:
            t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_footprint';t.header.stamp=stamp
            t.transform.translation.x=state['x'];t.transform.translation.y=state['y']
            t.transform.rotation.z=math.sin(state['yaw']/2);t.transform.rotation.w=math.cos(state['yaw']/2)
            broadcaster.sendTransform(t)
        if now-last_map>.1:
            last_map=now
            m=Costmap();m.header.frame_id='map';m.header.stamp=stamp
            m.metadata.resolution=.05;m.metadata.size_x=100;m.metadata.size_y=100
            m.metadata.origin.position.x=-2.5;m.metadata.origin.position.y=-2.5;m.metadata.origin.orientation.w=1.
            m.data=[254 if state['blocked'] else 0]*10000;cm.publish(m)
            poly=PolygonStamped();poly.header.frame_id='map';poly.header.stamp=stamp
            for x,y in [(-.205,-.145),(-.205,.145),(.077,.145),(.077,-.145)]:
                poly.polygon.points.append(Point32(x=state['x']+x*math.cos(state['yaw'])-y*math.sin(state['yaw']),
                    y=state['y']+x*math.sin(state['yaw'])+y*math.cos(state['yaw'])))
            fp.publish(poly)
    timer=node.create_timer(.02,tick)
    def until(predicate,seconds=10):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return
        raise RuntimeError('fixture timeout')
    def wait(seconds):
        end=time.monotonic()+seconds;until(lambda:time.monotonic()>=end,seconds+1)
    processes=[];handles=[]
    try:
        for pkg,exe in [('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]:
            handle=(out/(exe+'.log')).open('w');handles.append(handle)
            processes.append(subprocess.Popen(['ros2','run',pkg,exe,'--ros-args','--params-file',str(config),
                '-r','cmd_vel:=/handyman_test/turn/cmd_vel'],stdout=handle,stderr=subprocess.STDOUT))
        for name in ('behavior_server','bt_navigator'):
            client=node.create_client(ChangeState,'/'+name+'/change_state')
            until(lambda:client.service_is_ready(),15)
            for transition in (1,3):
                req=ChangeState.Request();req.transition.id=transition;future=client.call_async(req)
                until(future.done,20);assert future.result().success
        action=ActionClient(node,NavigateToPose,'/navigate_to_pose');until(action.server_is_ready,10)
        wait(.7)
        report=[]
        def send(delta=math.pi/4,dx=.104):
            goal=NavigateToPose.Goal();goal.behavior_tree=str(share/'behavior_trees/search_turn.xml')
            goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
            goal.pose.pose.position.x=state['x']+dx;goal.pose.pose.position.y=state['y']
            desired=state['yaw']+delta
            goal.pose.pose.orientation.z=math.sin(desired/2);goal.pose.pose.orientation.w=math.cos(desired/2)
            future=action.send_goal_async(goal);until(future.done)
            handle=future.result();assert handle.accepted
            return handle,desired
        for case in ('turn_3_to_4','wrap_pi','cancel','obstacle','stale_tf','outside_xy'):
            state['blocked']=case=='obstacle';wait(.4)
            start=time.monotonic();offset=len(commands)
            handle,target=send(dx=.2 if case=='outside_xy' else .104)
            result=handle.get_result_async()
            if case=='cancel':
                wait(.5);cancel=handle.cancel_goal_async();until(cancel.done);assert cancel.result().goals_canceling
            if case=='stale_tf':wait(.3);state['publish_tf']=False
            until(result.done,10);wait(.25)
            status=result.result().status;elapsed=time.monotonic()-start
            recorded=commands[offset:]
            assert recorded or case=='outside_xy'
            assert abs(state['w'])<1e-9
            assert all(abs(w)<=.450001 for _,w in recorded)
            if case in ('turn_3_to_4','wrap_pi'):
                error=abs(math.atan2(math.sin(target-state['yaw']),math.cos(target-state['yaw'])))
                assert status==4 and error<.10 and elapsed<5,(case,status,error,elapsed)
                assert all(w>=0 for _,w in recorded)
            elif case=='cancel':assert status==5,(case,status)
            else:assert status==6,(case,status)
            if case in ('obstacle','outside_xy'):assert all(w==0 for _,w in recorded)
            state['publish_tf']=True
            report.append(dict(case=case,status=status,seconds=elapsed,commands=recorded))
            print(case,status,round(elapsed,3),flush=True)
        (out/'result.json').write_text(json.dumps(report,indent=2))
        print('PASS '+str(out),flush=True)
    finally:
        for proc in processes:
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
        for proc in processes:
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
        for h in handles:h.close()
        node.destroy_node();rclpy.shutdown()
        print('Evidence: '+str(out),flush=True)

if __name__=='__main__':main()

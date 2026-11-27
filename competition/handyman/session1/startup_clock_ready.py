"""Verify initial joint clock, using the existing executable's native sync channel.

No arm/base/gripper commands. Does not rewrite ROS header timestamps.
"""
import argparse,json,os,time
from pathlib import Path

def ready_window(rows,now):
    if len(rows)<20 or rows[-1][0]-rows[0][0]<2.:return False
    if now-rows[-1][0]>.5:return False
    if any(b[1]<=a[1] for a,b in zip(rows,rows[1:])):return False
    return all(0<=x[2]<=.1 for x in rows)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='71':raise ValueError('existing_domain71_required')
    import rclpy
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init();node=rclpy.create_node('handyman_startup_clock_check');rows=[];actions=[]
    def receive(m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec;now=time.monotonic()
        if rows and (stamp<=rows[-1][1] or now-rows[-1][0]>.6):rows.clear()
        rows.append((now,stamp,(node.get_clock().now().nanoseconds-stamp)/1e9))
        while rows and now-rows[0][0]>3.:rows.pop(0)
    node.create_subscription(JointState,'/hsrb/joint_states',receive,qos_profile_sensor_data)
    service=node.create_client(Trigger,'/sigverse/recalibrate_startup_clock');started=time.monotonic();last_call=0.;last_log=0.;missing_since=None
    def save(status,reason=None):
        report=dict(status=status,reason=reason,elapsed_s=time.monotonic()-started,samples=len(rows),
            joint_age_range_s=[min(x[2] for x in rows),max(x[2] for x in rows)] if rows else None,
            source_clock_actions=actions,ros_headers_rewritten=False,robot_commands_sent=False)
        a.output.write_text(json.dumps(report,indent=2));return report
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05);now=time.monotonic()
            if ready_window(rows,now) and node.count_publishers('/hsrb/joint_states')==1:
                print(json.dumps(save('clock_ready')),flush=True);return
            if now-last_log>=5:
                print(json.dumps(save('waiting_for_source_clock')),flush=True);last_log=now
            if len(rows)<20 or now-last_call<2:continue
            if not service.service_is_ready():
                if missing_since is None:missing_since=now
                if now-missing_since>10:raise RuntimeError('automatic_clock_service_missing_upgrade_bridge_and_reconnect_exe')
                continue
            missing_since=None;last_call=now;future=service.call_async(Trigger.Request());deadline=time.monotonic()+4
            while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            if not future.done():raise RuntimeError('automatic_clock_service_response_missing')
            result=future.result();actions.append(dict(success=result.success,message=result.message))
            print(json.dumps(actions[-1]),flush=True)
            if result.success and result.message.startswith('source_clock_adjustment_sent'):rows.clear()
            elif not result.success and result.message in ('exactly_one_native_unity_timesync_connection_required','motion_owner_present_stop_before_clock_calibration'):
                raise RuntimeError(result.message)
    except BaseException as exc:
        save('failed',str(exc));raise
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()

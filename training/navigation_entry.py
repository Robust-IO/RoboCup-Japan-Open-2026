"""Restore idle Nav2 parameters before a goal; no robot goals or joint commands."""
import json
import math
import os
import subprocess
import time
import uuid
from pathlib import Path

PRECISION_NAMES=('general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','FollowPath.xy_goal_tolerance')
PRECISION=(.06,.08,.06)

def precision_matches(values):
    return len(values)==3 and all(v is not None and math.isfinite(v) and abs(v-e)<1e-8 for v,e in zip(values,PRECISION))


def finite_navigation_joints(joints,names):
    if any(name not in joints or not math.isfinite(joints[name]) for name in names):
        raise ValueError('invalid_navigation_joint_feedback')
    return True  # Pose/opening equality is deliberately not a navigation gate.

def ensure_idle_precision(adapter,*,wait,on_wait=lambda _:None,cancelled=lambda:False):
    """Service attempts are finite; aggregate repair is cancellable and unbounded."""
    while True:
        if cancelled():raise InterruptedError('navigation_repair_cancelled')
        try:
            state=adapter.state()
            if state not in (1,2):
                on_wait('navigation_not_idle_waiting_for_current_goal');wait(.25);continue
            if state==2:
                adapter.cleanup();continue
            values=adapter.parameters()
            if precision_matches(values):
                return dict(status='navigation_precision_ready',values=list(values),robot_commands_sent=False)
            if state==1:adapter.configure()
            adapter.set_parameters(PRECISION)
            adapter.cleanup()
            # Verify from service feedback on the next iteration, not from intended writes.
        except (RuntimeError,TimeoutError,OSError) as exc:
            on_wait(str(exc))
            adapter.restore_missing_services()
            wait(.25)

class ROSNavigationAdapter:
    def __init__(self,node):
        self.node=node;self.started={}
    def call(self,kind,suffix,request,timeout=5):
        import rclpy
        client=self.node.create_client(kind,'/controller_server/'+suffix)
        try:
            if not client.wait_for_service(timeout_sec=.5):raise RuntimeError('navigation_service_missing:'+suffix)
            future=client.call_async(request);end=time.monotonic()+timeout
            while not future.done() and time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.02)
            if not future.done():raise TimeoutError('navigation_service_wait:'+suffix)
            if future.exception():raise RuntimeError(str(future.exception()))
            return future.result()
        finally:self.node.destroy_client(client)
    def state(self):
        from lifecycle_msgs.srv import GetState
        return self.call(GetState,'get_state',GetState.Request()).current_state.id
    def parameters(self):
        from rcl_interfaces.srv import GetParameters
        req=GetParameters.Request();req.names=list(PRECISION_NAMES)
        return [v.double_value if v.type==3 else None for v in self.call(GetParameters,'get_parameters',req).values]
    def transition(self,identifier):
        from lifecycle_msgs.srv import ChangeState
        req=ChangeState.Request();req.transition.id=identifier
        if not self.call(ChangeState,'change_state',req,timeout=15).success:raise RuntimeError('navigation_initialization_transition_failed')
    def configure(self):self.transition(1)
    def cleanup(self):self.transition(2)
    def set_parameters(self,values):
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        req=SetParameters.Request();req.parameters=[Parameter(n,value=float(v)).to_parameter_msg() for n,v in zip(PRECISION_NAMES,values)]
        result=self.call(SetParameters,'set_parameters',req)
        if not all(v.successful for v in result.results):raise RuntimeError('navigation_parameter_repair_rejected')
    def restore_missing_services(self):
        """Restore absent known nodes; never kill existing processes or start a goal."""
        from ament_index_python.packages import get_package_share_directory
        import yaml
        share=Path(get_package_share_directory('handyman_ros2'));rebuilt=Path(get_package_share_directory('handyman_rebuild_ros2'))
        params=share/'param/nav2_params.yaml';bt=share/'behavior_trees/navigate_w_recovery.xml'
        commands={
          'map_server':['ros2','run','nav2_map_server','map_server','--ros-args','-p','yaml_filename:='+str(rebuilt/'maps/LayoutA/map.yaml')],
          'lifecycle_manager_map':['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-r','__node:=lifecycle_manager_map','-p','autostart:=true','-p',"node_names:=['map_server']"],
          'lifecycle_manager_navigation':['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-r','__node:=lifecycle_manager_navigation','-p','autostart:=false','-p',"node_names:=['controller_server','planner_server','behavior_server','bt_navigator']"]}
        for package,name in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]:
            cmd=['ros2','run',package,name,'--ros-args','--params-file',str(params),'-r','cmd_vel:=/hsrb/command_velocity']
            if name=='controller_server':
                for key,value in zip(PRECISION_NAMES,PRECISION):cmd+=['-p',key+':='+str(value)]
            if name=='bt_navigator':cmd+=['-p','default_nav_to_pose_bt_xml:='+str(bt)]
            commands[name]=cmd
        nodes=set(self.node.get_node_names())
        # DDS discovery can lag. Check the exact executable/node remap too.
        process_args=[]
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:args=[v.decode(errors='replace') for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
            except (FileNotFoundError,PermissionError):continue
            if args:process_args.append(args)
        for name,cmd in commands.items():
            if name in nodes:continue
            exists=any(Path(a[0]).name==name or ('__node:='+name) in a for a in process_args)
            if exists:continue
            prior=self.started.get(name)
            if prior is not None and prior.poll() is None:continue
            root=Path('/mnt/d/handyman-training-runs')/'navigation-recovery';root.mkdir(exist_ok=True)
            log=root/(name+'-'+uuid.uuid4().hex[:8]+'.log')
            with log.open('x') as stream:child=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            self.started[name]=child
            (root/(name+'-latest.json')).write_text(json.dumps(dict(pid=child.pid,command=cmd,log=str(log),robot_commands_sent=False),indent=2))

def repair_navigation(node,*,root=None,record=lambda _:None):
    import rclpy
    # Import only at runtime; this module also serves the frozen source launcher.
    from gate_recovery import RecoveryStatus
    status=RecoveryStatus(root,record)
    adapter=ROSNavigationAdapter(node)
    adapter.restore_missing_services()
    result=ensure_idle_precision(adapter,wait=lambda dt:rclpy.spin_once(node,timeout_sec=dt),on_wait=status.waiting,cancelled=lambda:not rclpy.ok())
    status.recovered();return result
